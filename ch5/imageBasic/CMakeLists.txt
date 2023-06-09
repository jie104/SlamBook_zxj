cmake_minimum_required(VERSION 2.8)
PROJECT(imageBasic)

#添加C++ 11标准支持
set(CMKAE_CXX_FLAGS "-std=c++11")
include_directories("/usr/local/opencv3")
#寻找opencv库
find_package(OpenCV REQUIRED)

#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasic imageBasic.cpp)

add_executable(undistortImage undistortImage.cpp)

target_link_libraries(
        undistortImage
        ${CATKIN_LIBRARIES}
        ${OpenCV_LIBS}
)

target_link_libraries(
        imageBasic
        ${CATKIN_LIBRARIES}
        ${OpenCV_LIBS}
)