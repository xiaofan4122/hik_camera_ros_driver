#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include <iostream>
#include <string>

using namespace std;

class HikCameraNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    void* handle_;
    unsigned char* pData_;
    unsigned char* pDataForRGB_;
    unsigned int nDataSize_;
    unsigned int nDataForRGBSize_;
    MV_CC_DEVICE_INFO_LIST stDeviceList_;

    std::string topic_name_;

    // --- 时间同步相关变量 ---
    bool use_ptp_;
    ros::Time boot_time_offset_;
    bool is_offset_calculated_;

public:
    HikCameraNode() : nh_(), pnh_("~"), it_(nh_), handle_(NULL), pData_(NULL), pDataForRGB_(NULL),
                      use_ptp_(false), is_offset_calculated_(false) {
        pnh_.param<std::string>("image_topic", topic_name_, "/hik_camera/image_raw");
        printUsageHint();
        image_pub_ = it_.advertise(topic_name_, 1);
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

    void printUsageHint() {
        const char* GREEN  = "\033[32m";
        const char* YELLOW = "\033[33m";
        const char* RESET  = "\033[0m";
        const char* BOLD   = "\033[1m";

        ROS_INFO("%s-----------------------------------------------------------------------%s", GREEN, RESET);
        ROS_INFO("%s  Hikvision Camera Node Started%s", GREEN, RESET);
        ROS_INFO("%s  Usage: %srosrun hik_camera_ros_driver hik_camera_node _user_set:=<ID> _image_topic:=<NAME>%s", GREEN, YELLOW, RESET);
        ROS_INFO("%s  Params:%s", GREEN, RESET);
        ROS_INFO("%s    _user_set:   -1 (Default), 0 (Factory), 1-3 (UserSet)%s", YELLOW, RESET);
        ROS_INFO("%s    _image_topic: Output topic name (Default: /hik_camera/image_raw)%s", YELLOW, RESET);
        ROS_INFO("%s-----------------------------------------------------------------------%s", GREEN, RESET);
        ROS_INFO("%s  Current Topic: %s%s%s", GREEN, BOLD, topic_name_.c_str(), RESET);
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
        if (MV_OK != nRet) { ROS_ERROR("Create Handle failed!"); return; }

        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet) {
            string errDesc;
            switch((unsigned int)nRet) {
                case 0x80000004: errDesc = "MV_E_RESOURCE (Resource in use)"; break;
                case 0x80000005: errDesc = "MV_E_ACCESS (Access denied / IP mismatch)"; break;
                case 0x80000203: errDesc = "MV_E_ACCESSIBILITY (Device busy / Restarting / Unreachable)"; break;
                case 0x80000206: errDesc = "MV_E_NETER (Network error / Firewall)"; break;
                default:         errDesc = "Unknown Error"; break;
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

        pData_ = (unsigned char*)malloc(nDataSize_);
        nDataForRGBSize_ = nDataSize_ * 3 + 2048;
        pDataForRGB_ = (unsigned char*)malloc(nDataForRGBSize_);

        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet) {
            ROS_ERROR("Start Grabbing failed! Ret: [%x]", nRet);
            return;
        }
        ROS_INFO("Hikvision Camera Initialized. Publishing to [%s]", topic_name_.c_str());
    }

    void loadUserSet(int set_id) {
        if (set_id < 0) return;
        int nRet = MV_CC_SetEnumValue(handle_, "UserSetSelector", set_id);
        if (nRet != MV_OK) { ROS_WARN("Failed to select UserSet %d!", set_id); return; }

        nRet = MV_CC_SetCommandValue(handle_, "UserSetLoad");
        if (nRet != MV_OK) ROS_WARN("Failed to load UserSet %d!", set_id);
        else ROS_INFO("Config: Successfully loaded UserSet [%d].", set_id);
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
        switch(ptp_status_enum.nCurValue) {
            case 0: status_str = "Initializing"; break;
            case 2: status_str = "Disabled"; break;
            case 3: status_str = "Listening"; break;
            case 5: status_str = "Master"; break;
            case 8: status_str = "Slave"; break;
            default: status_str = to_string(ptp_status_enum.nCurValue); break;
        }

        if (ret_enable == MV_OK && ret_status == MV_OK) {
            if (ptp_enable && ptp_status_enum.nCurValue == 8) {
                return {true, status_str};
            }
        }
        return {false, status_str};
    }

    void run() {
        if (!handle_) return;
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

        ros::Time last_log_time = ros::Time::now();
        int frame_count = 0;
        uint64_t last_timestamp = 0;

        // 设定一个阈值，比如 2020年 (单位: 纳秒)
        // 1577836800 * 1e9
        const uint64_t ABSOLUTE_TIME_THRESHOLD = 1600000000000000000ULL;

        while (ros::ok()) {
            int nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, nDataSize_, &stImageInfo, 1000);

            if (nRet == MV_OK) {
                frame_count++;

                uint64_t dev_time_ns = ((uint64_t)stImageInfo.nDevTimeStampHigh << 32) | stImageInfo.nDevTimeStampLow;
                last_timestamp = dev_time_ns;

                // ==========================================================
                // 1. PTP 状态检测
                // ==========================================================
                pair<bool, string> ptp_status = checkPtpLockedStatus();
                bool should_hard_sync = ptp_status.first;

                // ==========================================================
                // 2. 启发式检测 (Heuristic Check) - 防止状态滞后导致的翻倍
                // ==========================================================
                // 如果传入的时间戳已经是 Unix 绝对时间，无论状态如何，都强制硬同步
                bool is_timestamp_absolute = (dev_time_ns > ABSOLUTE_TIME_THRESHOLD);

                if (is_timestamp_absolute) {
                    should_hard_sync = true; // 强制覆盖
                }

                // ==========================================================
                // 3. 状态切换与 Offset 重置
                // ==========================================================
                if (should_hard_sync != use_ptp_) {
                    if (should_hard_sync) {
                         if (is_timestamp_absolute) {
                             // 因数据本身触发的切换
                             ROS_WARN("\033[32m[SYNC SWITCH] Timestamp looks like Unix Time! Enforcing Hard Sync.\033[0m");
                         } else {
                             // 因 PTP 状态触发的切换
                             ROS_WARN("\033[32m[SYNC SWITCH] PTP Locked (State: %s)! Switching to Hard Sync.\033[0m", ptp_status.second.c_str());
                         }
                    } else {
                         ROS_WARN("\033[33m[SYNC SWITCH] PTP Lost. Switching to Soft Sync.\033[0m");
                         is_offset_calculated_ = false;
                    }
                    use_ptp_ = should_hard_sync;
                }

                // ==========================================================
                // 4. 计算最终时间戳
                // ==========================================================
                ros::Time image_timestamp;

                if (use_ptp_) {
                    // 硬同步
                    image_timestamp.fromNSec(dev_time_ns);
                } else {
                    // 软同步
                    if (!is_offset_calculated_) {
                        boot_time_offset_ = ros::Time::now() - ros::Duration(dev_time_ns / 1e9);
                        is_offset_calculated_ = true;
                    }
                    image_timestamp = boot_time_offset_ + ros::Duration(dev_time_ns / 1e9);
                }

                stConvertParam.nWidth = stImageInfo.nWidth;
                stConvertParam.nHeight = stImageInfo.nHeight;
                stConvertParam.pSrcData = pData_;
                stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
                stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
                stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
                stConvertParam.pDstBuffer = pDataForRGB_;
                stConvertParam.nDstBufferSize = nDataForRGBSize_;

                int nConvertRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
                if (MV_OK == nConvertRet) {
                    cv::Mat cv_image(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB_);
                    std_msgs::Header header;
                    header.stamp = image_timestamp;
                    header.frame_id = "camera_optical_frame";

                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "rgb8", cv_image).toImageMsg();
                    image_pub_.publish(msg);
                }
            }

            // 监控日志
            ros::Time now = ros::Time::now();
            double elapsed = (now - last_log_time).toSec();
            if (elapsed >= 2.0) {
                double actual_fps = frame_count / elapsed;
                MVCC_FLOATVALUE stExp, stGain;
                float current_exp = 0.0f;
                float current_gain = 0.0f;

                if (MV_CC_GetFloatValue(handle_, "ExposureTime", &stExp) == MV_OK) current_exp = stExp.fCurValue;
                if (MV_CC_GetFloatValue(handle_, "Gain", &stGain) == MV_OK)        current_gain = stGain.fCurValue;

                string sync_mode = use_ptp_ ? "Hard(PTP)" : "Soft";

                ROS_INFO("[MONITOR] FPS: %.3f | Exp: %.3f us | Gain: %.3f dB | TS: %llu | Sync: %s",
                         actual_fps, current_exp, current_gain, (unsigned long long)last_timestamp, sync_mode.c_str());

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