cmake_minimum_required(VERSION 2.8.3)
project(ur-ros-driver-config)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
            roscpp
            sensor_msgs
            std_msgs
            cv_bridge
            image_transport
            )

find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

find_package(OpenCV REQUIRED)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

catkin_package(
    CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    image_transport
    )

include_directories(${catkin_INCLUDE_DIRS} ${realsense_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
add_subdirectory(src/basic)