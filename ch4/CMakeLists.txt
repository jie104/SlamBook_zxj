cmake_minimum_required(VERSION 2.8)

project(useSophus)

#find_package(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})
link_directories("/usr/local/lib")

add_executable(useSophus useSophus.cpp)
add_executable(trajectoryError example/trajectoryError.cpp)

target_link_libraries(useSophus
        ${catkin_LIBRARIES}
        Sophus
        )

target_link_libraries(trajectoryError
        ${catkin_LIBRARIES}
        Sophus
        pangolin
        GL
        )
