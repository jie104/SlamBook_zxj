cmake_minimum_required(VERSION 2.8)
project(jointMap)

find_package(catkin REQUIRED COMPONENTS
        Pangolin
        OpenCV
        Sophus)
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#find_package(Sophus )
#include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})

link_directories("/usr/local/lib")


add_executable(${PROJECT_NAME}_node jointMap.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
#        Sophus
#        ${OpenCV_LIBS}
#        ${Pangolin_LIBRARIES}
        GL
        )


