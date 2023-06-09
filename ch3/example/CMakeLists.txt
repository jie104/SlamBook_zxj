cmake_minimum_required(VERSION 2.8)

project(example)
#find_package(pangolin REQUIRED)
include_directories(${pangolin_INCLUDE_DIRS})
LINK_directories("/usr/local/lib")

add_executable(coordinateTransform coordinateTransform.cpp)

add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory
        ${catkin_LIBRARIES}
        GL
        pangolin)


