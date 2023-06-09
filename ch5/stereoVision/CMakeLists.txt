cmake_minimum_required(VERSION 2.8)

project(stereoVision)
set(CMKAE_CXX_FLAGS "-std=c++11")

include_directories("/usr/local/")
find_package(Pangolin REQUIRED)
include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED)

include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(
        stereoVision
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
)


