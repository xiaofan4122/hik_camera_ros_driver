#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>

#include "MvCameraControl.h"
#include "hik_camera_ros_driver/HikFrame.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <utility>

using namespace std;

class HikCameraNode {
private:
    enum PublishMode {
        SPLIT_TOPICS,
        PACKED
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher exposure_pub_;
    ros::Publisher gain_pub_;
    ros::Publisher device_timestamp_pub_;
    ros::Publisher packed_frame_pub_;

    void* handle_;
    unsigned char* pData_;
    unsigned char* pDataForRGB_;
    unsigned int nDataSize_;
    unsigned int nDataForRGBSize_;
    MV_CC_DEVICE_INFO_LIST stDeviceList_;

    string image_topic_;
    string exposure_topic_;
    string gain_topic_;
    string device_timestamp_topic_;
    string packed_topic_;
    string frame_id_;
    PublishMode publish_mode_;

    bool use_ptp_;
    ros::Time boot_time_offset_;
    bool is_offset_calculated_;
    bool has_warned_missing_chunk_metadata_;

public:
    HikCameraNode()
        : nh_(),
          pnh_("~"),
          it_(nh_),
          handle_(NULL),
          pData_(NULL),
          pDataForRGB_(NULL),
          nDataSize_(0),
          nDataForRGBSize_(0),
          publish_mode_(SPLIT_TOPICS),
          use_ptp_(false),
          is_offset_calculated_(false),
          has_warned_missing_chunk_metadata_(false) {
        pnh_.param<string>("image_topic", image_topic_, "/hik_camera/image_raw");
        pnh_.param<string>("exposure_topic", exposure_topic_, "/hik_camera/exposure_time");
        pnh_.param<string>("gain_topic", gain_topic_, "/hik_camera/gain");
        pnh_.param<string>("device_timestamp_topic", device_timestamp_topic_, "/hik_camera/device_timestamp");
        pnh_.param<string>("packed_topic", packed_topic_, "/hik_camera/frame");
        pnh_.param<string>("frame_id", frame_id_, "camera_optical_frame");
        publish_mode_ = parsePublishMode();

        printUsageHint();
        setupPublishers();
        initCamera();
    }

    ~HikCameraNode() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
        if (pData_) free(pData_);
        if (pDataForRGB_) free(pDataForRGB_);
        ROS_INFO("Hikvision Camera released.");
    }

    PublishMode parsePublishMode() {
        string publish_mode_str;
        pnh_.param<string>("publish_mode", publish_mode_str, "split_topics");

        if (publish_mode_str == "packed") {
            return PACKED;
        }

        if (publish_mode_str != "split_topics") {
            ROS_WARN("Unknown publish_mode [%s], fallback to [split_topics].", publish_mode_str.c_str());
        }
        return SPLIT_TOPICS;
    }

    void setupPublishers() {
        if (publish_mode_ == PACKED) {
            packed_frame_pub_ = nh_.advertise<hik_camera_ros_driver::HikFrame>(packed_topic_, 1);
            return;
        }

        image_pub_ = it_.advertise(image_topic_, 1);
        exposure_pub_ = nh_.advertise<std_msgs::Float32>(exposure_topic_, 1);
        gain_pub_ = nh_.advertise<std_msgs::Float32>(gain_topic_, 1);
        device_timestamp_pub_ = nh_.advertise<std_msgs::UInt64>(device_timestamp_topic_, 1);
    }

    void printUsageHint() {
        const char* GREEN = "\033[32m";
        const char* YELLOW = "\033[33m";
        const char* RESET = "\033[0m";
        const char* BOLD = "\033[1m";

        ROS_INFO("%s-----------------------------------------------------------------------%s", GREEN, RESET);
        ROS_INFO("%s  Hikvision Camera Node Started%s", GREEN, RESET);
        ROS_INFO("%s  Usage: %srosrun hik_camera_ros_driver hik_camera_node _user_set:=<ID> _publish_mode:=<MODE>%s", GREEN, YELLOW, RESET);
        ROS_INFO("%s  Params:%s", GREEN, RESET);
        ROS_INFO("%s    _user_set:             -1 (Default), 0 (Factory), 1-3 (UserSet)%s", YELLOW, RESET);
        ROS_INFO("%s    _publish_mode:         split_topics | packed%s", YELLOW, RESET);
        ROS_INFO("%s    _image_topic:          Image topic in split_topics mode%s", YELLOW, RESET);
        ROS_INFO("%s    _exposure_topic:       Exposure topic in split_topics mode%s", YELLOW, RESET);
        ROS_INFO("%s    _gain_topic:           Gain topic in split_topics mode%s", YELLOW, RESET);
        ROS_INFO("%s    _device_timestamp_topic: Device timestamp topic in split_topics mode%s", YELLOW, RESET);
        ROS_INFO("%s    _packed_topic:         Custom message topic in packed mode%s", YELLOW, RESET);
        ROS_INFO("%s-----------------------------------------------------------------------%s", GREEN, RESET);
        if (publish_mode_ == PACKED) {
            ROS_INFO("%s  Current Mode: %sPACKED%s -> %s%s%s", GREEN, BOLD, RESET, BOLD, packed_topic_.c_str(), RESET);
        } else {
            ROS_INFO("%s  Current Mode: %sSPLIT_TOPICS%s", GREEN, BOLD, RESET);
            ROS_INFO("%s  Image Topic:  %s%s%s", GREEN, BOLD, image_topic_.c_str(), RESET);
            ROS_INFO("%s  Exp Topic:    %s%s%s", GREEN, BOLD, exposure_topic_.c_str(), RESET);
            ROS_INFO("%s  Gain Topic:   %s%s%s", GREEN, BOLD, gain_topic_.c_str(), RESET);
            ROS_INFO("%s  TS Topic:     %s%s%s", GREEN, BOLD, device_timestamp_topic_.c_str(), RESET);
        }
        ROS_INFO("%s-----------------------------------------------------------------------%s", GREEN, RESET);
    }

    void initCamera() {
        int nRet = MV_OK;

        memset(&stDeviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList_);
        if (MV_OK != nRet || stDeviceList_.nDeviceNum == 0) {
            ROS_ERROR("No Hikvision camera found! Ret: [%x]", nRet);
            return;
        }

        ROS_INFO("Found %d devices. Connecting to the first one...", stDeviceList_.nDeviceNum);

        nRet = MV_CC_CreateHandle(&handle_, stDeviceList_.pDeviceInfo[0]);
        if (MV_OK != nRet) {
            ROS_ERROR("Create Handle failed!");
            return;
        }

        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet) {
            string errDesc;
            switch ((unsigned int)nRet) {
                case 0x80000004: errDesc = "MV_E_RESOURCE (Resource in use)"; break;
                case 0x80000005: errDesc = "MV_E_ACCESS (Access denied / IP mismatch)"; break;
                case 0x80000203: errDesc = "MV_E_ACCESSIBILITY (Device busy / Restarting / Unreachable)"; break;
                case 0x80000206: errDesc = "MV_E_NETER (Network error / Firewall)"; break;
                default: errDesc = "Unknown Error"; break;
            }
            ROS_ERROR("Open Device failed! Ret: [0x%x] -> %s", nRet, errDesc.c_str());

            if (nRet == 0x80000004) {
                ROS_ERROR("SOLUTION: The camera is occupied. CLOSE 'MVS' client.");
            } else {
                ROS_ERROR("SOLUTION: Check device status in '/opt/MVS/bin/MVS.sh'.");
            }

            MV_CC_DestroyHandle(handle_);
            handle_ = NULL;
            return;
        }

        int user_set_id = -1;
        pnh_.param("user_set", user_set_id, -1);
        loadUserSet(user_set_id);

        printCameraConfig();

        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
        nDataSize_ = stParam.nCurValue;

        pData_ = static_cast<unsigned char*>(malloc(nDataSize_));
        nDataForRGBSize_ = nDataSize_ * 3 + 2048;
        pDataForRGB_ = static_cast<unsigned char*>(malloc(nDataForRGBSize_));

        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet) {
            ROS_ERROR("Start Grabbing failed! Ret: [%x]", nRet);
            return;
        }

        if (publish_mode_ == PACKED) {
            ROS_INFO("Hikvision Camera Initialized. Publishing packed frames to [%s]", packed_topic_.c_str());
        } else {
            ROS_INFO("Hikvision Camera Initialized. Publishing image to [%s]", image_topic_.c_str());
        }
    }

    void loadUserSet(int set_id) {
        if (set_id < 0) return;
        int nRet = MV_CC_SetEnumValue(handle_, "UserSetSelector", set_id);
        if (nRet != MV_OK) {
            ROS_WARN("Failed to select UserSet %d!", set_id);
            return;
        }

        nRet = MV_CC_SetCommandValue(handle_, "UserSetLoad");
        if (nRet != MV_OK) {
            ROS_WARN("Failed to load UserSet %d!", set_id);
        } else {
            ROS_INFO("Config: Successfully loaded UserSet [%d].", set_id);
        }
    }

    void printCameraConfig() {
        int nRet;
        MVCC_INTVALUE stIntVal;
        MVCC_FLOATVALUE stFloatVal;
        MVCC_ENUMVALUE stEnumVal;

        ROS_INFO("========== Camera Configuration ==========");

        MV_CC_GetIntValue(handle_, "Width", &stIntVal);
        int width = stIntVal.nCurValue;
        MV_CC_GetIntValue(handle_, "Height", &stIntVal);
        int height = stIntVal.nCurValue;
        ROS_INFO("  Resolution   : %d x %d", width, height);

        nRet = MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stFloatVal);
        if (nRet == MV_OK) ROS_INFO("  Expected FPS : %.3f Hz", stFloatVal.fCurValue);

        string expMode = "Unknown";
        nRet = MV_CC_GetEnumValue(handle_, "ExposureAuto", &stEnumVal);
        if (nRet == MV_OK) {
            if (stEnumVal.nCurValue == 2) expMode = "Auto (Continuous)";
            else if (stEnumVal.nCurValue == 1) expMode = "Auto (Once)";
            else expMode = "Manual (Off)";
        }
        nRet = MV_CC_GetFloatValue(handle_, "ExposureTime", &stFloatVal);
        if (nRet == MV_OK) ROS_INFO("  Exposure     : %.3f us | Mode: %s", stFloatVal.fCurValue, expMode.c_str());

        string gainMode = "Unknown";
        nRet = MV_CC_GetEnumValue(handle_, "GainAuto", &stEnumVal);
        if (nRet == MV_OK) {
            if (stEnumVal.nCurValue == 2) gainMode = "Auto (Continuous)";
            else if (stEnumVal.nCurValue == 1) gainMode = "Auto (Once)";
            else gainMode = "Manual (Off)";
        }
        nRet = MV_CC_GetFloatValue(handle_, "Gain", &stFloatVal);
        if (nRet == MV_OK) ROS_INFO("  Gain         : %.3f dB | Mode: %s", stFloatVal.fCurValue, gainMode.c_str());

        nRet = MV_CC_GetEnumValue(handle_, "TriggerMode", &stEnumVal);
        if (nRet == MV_OK) {
            string triggerMode = (stEnumVal.nCurValue == 0) ? "Off (Continuous)" : "On (Hardware/Software)";
            ROS_INFO("  TriggerMode  : %s", triggerMode.c_str());
        }

        ROS_INFO("==========================================");
    }

    pair<bool, string> checkPtpLockedStatus() {
        bool ptp_enable = false;
        MVCC_ENUMVALUE ptp_status_enum = {0};

        int ret_enable = MV_CC_GetBoolValue(handle_, "GevIEEE1588", &ptp_enable);
        int ret_status = MV_CC_GetEnumValue(handle_, "GevIEEE1588Status", &ptp_status_enum);

        string status_str = "Unknown";
        switch (ptp_status_enum.nCurValue) {
            case 0: status_str = "Initializing"; break;
            case 2: status_str = "Disabled"; break;
            case 3: status_str = "Listening"; break;
            case 5: status_str = "Master"; break;
            case 8: status_str = "Slave"; break;
            default: status_str = to_string(ptp_status_enum.nCurValue); break;
        }

        if (ret_enable == MV_OK && ret_status == MV_OK) {
            if (ptp_enable && ptp_status_enum.nCurValue == 8) {
                return make_pair(true, status_str);
            }
        }
        return make_pair(false, status_str);
    }

    ros::Time computeImageTimestamp(uint64_t dev_time_ns) {
        const uint64_t ABSOLUTE_TIME_THRESHOLD = 1600000000000000000ULL;

        pair<bool, string> ptp_status = checkPtpLockedStatus();
        bool should_hard_sync = ptp_status.first;
        bool is_timestamp_absolute = (dev_time_ns > ABSOLUTE_TIME_THRESHOLD);

        if (is_timestamp_absolute) {
            should_hard_sync = true;
        }

        if (should_hard_sync != use_ptp_) {
            if (should_hard_sync) {
                if (is_timestamp_absolute) {
                    ROS_WARN("\033[32m[SYNC SWITCH] Timestamp looks like Unix Time! Enforcing Hard Sync.\033[0m");
                } else {
                    ROS_WARN("\033[32m[SYNC SWITCH] PTP Locked (State: %s)! Switching to Hard Sync.\033[0m", ptp_status.second.c_str());
                }
            } else {
                ROS_WARN("\033[33m[SYNC SWITCH] PTP Lost. Switching to Soft Sync.\033[0m");
                is_offset_calculated_ = false;
            }
            use_ptp_ = should_hard_sync;
        }

        ros::Time image_timestamp;
        if (use_ptp_) {
            image_timestamp.fromNSec(dev_time_ns);
        } else {
            if (!is_offset_calculated_) {
                boot_time_offset_ = ros::Time::now() - ros::Duration(dev_time_ns / 1e9);
                is_offset_calculated_ = true;
            }
            image_timestamp = boot_time_offset_ + ros::Duration(dev_time_ns / 1e9);
        }
        return image_timestamp;
    }

    void warnIfChunkMetadataMissing(float exposure_time_us, float gain_db) {
        if (has_warned_missing_chunk_metadata_) {
            return;
        }

        if (exposure_time_us <= 0.0f && gain_db <= 0.0f) {
            ROS_WARN("\033[33m[CHUNK DATA] Failed to read per-frame exposure/gain metadata. Enable Data Chunk in MVS to output these fields with each frame.\033[0m");
            has_warned_missing_chunk_metadata_ = true;
        }
    }

    void publishSplitTopics(const sensor_msgs::ImagePtr& image_msg, uint64_t device_timestamp_ns,
                            float exposure_time_us, float gain_db) {
        std_msgs::Float32 exposure_msg;
        exposure_msg.data = exposure_time_us;

        std_msgs::Float32 gain_msg;
        gain_msg.data = gain_db;

        std_msgs::UInt64 device_timestamp_msg;
        device_timestamp_msg.data = device_timestamp_ns;

        image_pub_.publish(image_msg);
        exposure_pub_.publish(exposure_msg);
        gain_pub_.publish(gain_msg);
        device_timestamp_pub_.publish(device_timestamp_msg);
    }

    void publishPackedFrame(const sensor_msgs::ImagePtr& image_msg, uint64_t device_timestamp_ns,
                            float exposure_time_us, float gain_db) {
        hik_camera_ros_driver::HikFrame frame_msg;
        frame_msg.image = *image_msg;
        frame_msg.device_timestamp_ns = device_timestamp_ns;
        frame_msg.exposure_time_us = exposure_time_us;
        frame_msg.gain_db = gain_db;
        packed_frame_pub_.publish(frame_msg);
    }

    void run() {
        if (!handle_) return;

        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

        ros::Time last_log_time = ros::Time::now();
        int frame_count = 0;
        uint64_t last_timestamp = 0;

        while (ros::ok()) {
            int nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, nDataSize_, &stImageInfo, 1000);

            if (nRet == MV_OK) {
                frame_count++;

                uint64_t dev_time_ns = (static_cast<uint64_t>(stImageInfo.nDevTimeStampHigh) << 32) |
                                       stImageInfo.nDevTimeStampLow;
                last_timestamp = dev_time_ns;
                ros::Time image_timestamp = computeImageTimestamp(dev_time_ns);

                stConvertParam.nWidth = stImageInfo.nWidth;
                stConvertParam.nHeight = stImageInfo.nHeight;
                stConvertParam.pSrcData = pData_;
                stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
                stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
                stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
                stConvertParam.pDstBuffer = pDataForRGB_;
                stConvertParam.nDstBufferSize = nDataForRGBSize_;

                int nConvertRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
                if (nConvertRet == MV_OK) {
                    cv::Mat cv_image(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB_);
                    std_msgs::Header header;
                    header.stamp = image_timestamp;
                    header.frame_id = frame_id_;

                    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "rgb8", cv_image).toImageMsg();
                    warnIfChunkMetadataMissing(stImageInfo.fExposureTime, stImageInfo.fGain);

                    if (publish_mode_ == PACKED) {
                        publishPackedFrame(image_msg, dev_time_ns, stImageInfo.fExposureTime, stImageInfo.fGain);
                    } else {
                        publishSplitTopics(image_msg, dev_time_ns, stImageInfo.fExposureTime, stImageInfo.fGain);
                    }
                }
            }

            ros::Time now = ros::Time::now();
            double elapsed = (now - last_log_time).toSec();
            if (elapsed >= 2.0) {
                double actual_fps = frame_count / elapsed;
                MVCC_FLOATVALUE stExp, stGain;
                float current_exp = 0.0f;
                float current_gain = 0.0f;

                if (MV_CC_GetFloatValue(handle_, "ExposureTime", &stExp) == MV_OK) current_exp = stExp.fCurValue;
                if (MV_CC_GetFloatValue(handle_, "Gain", &stGain) == MV_OK) current_gain = stGain.fCurValue;

                string sync_mode = use_ptp_ ? "Hard(PTP)" : "Soft";

                ROS_INFO("[MONITOR] FPS: %.3f | Exp: %.3f us | Gain: %.3f dB | TS: %llu | Sync: %s",
                         actual_fps, current_exp, current_gain, static_cast<unsigned long long>(last_timestamp), sync_mode.c_str());

                frame_count = 0;
                last_log_time = now;
            }

            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hik_camera_node");
    HikCameraNode cam;
    cam.run();
    return 0;
}
