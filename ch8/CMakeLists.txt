cmake_minimum_required(VERSION 2.8)
project(ch8)

#set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)

set(OpenCV_DIR /usr/include/opencv4/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mpopcnt")
find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        OpenCV
#        Sophus
        )

list(APPEND CMAKE_MODULE_PATH  /home/zxj/lib/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)
find_package( G2O REQUIRED )
include_directories(
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3" )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(optical_flow optical_flow.cpp)


target_link_libraries(optical_flow
        ${catkin_LIBRARIES}
        Sophus
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
#        OpenCV
        )

