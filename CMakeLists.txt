cmake_minimum_required(VERSION 3.10)

project(hik_camera_ros_driver)

set(CATKIN_ENABLE_TESTING OFF)

# 1. 基础依赖查找
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)
find_package(OpenCV REQUIRED)

# =============================================================
#                 海康 SDK 自动查找逻辑
# =============================================================
set(MVS_ROOT_DIR "/opt/MVS")

# 自动检测系统架构
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(MVS_ARCH_DIR "64")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(MVS_ARCH_DIR "aarch64")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    set(MVS_ARCH_DIR "armhf")
else()
    set(MVS_ARCH_DIR "32")
endif()

# 查找头文件路径
find_path(MVS_INCLUDE_DIR MvCameraControl.h
    HINTS
    ${MVS_ROOT_DIR}/include
    /usr/include
    /usr/local/include
)

# 查找动态库文件
find_library(MVS_LIBRARY MvCameraControl
    HINTS
    ${MVS_ROOT_DIR}/lib/${MVS_ARCH_DIR}
    ${MVS_ROOT_DIR}/lib/64
    ${MVS_ROOT_DIR}/lib/32
    /usr/lib
    /usr/local/lib
)

if (MVS_INCLUDE_DIR AND MVS_LIBRARY)
    message(STATUS "[${PROJECT_NAME}] Found SDK Lib: ${MVS_LIBRARY}")
else()
    message(FATAL_ERROR "[${PROJECT_NAME}] MVS SDK not found!")
endif()

# =============================================================
#                        编译设置
# =============================================================

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${MVS_INCLUDE_DIR}
)

add_executable(hik_camera_node src/hik_camera_node.cpp
        src/hik_camera_node.cpp)

target_link_libraries(hik_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MVS_LIBRARY}
  X11
  pthread
  dl
)
