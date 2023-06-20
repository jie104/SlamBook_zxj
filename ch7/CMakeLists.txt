cmake_minimum_required(VERSION 2.8)
project(ch7)

set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mpopcnt")

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

add_executable(orb_cv orb_cv.cpp)

add_executable(orb_self orb_self.cpp)

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)

add_executable(triangulation triangulation.cpp)

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)

target_link_libraries(orb_cv
        ${catkin_LIBRARIES}
        )

target_link_libraries(orb_self
        ${catkin_LIBRARIES}
        )

target_link_libraries(pose_estimation_2d2d
        ${catkin_LIBRARIES}
        )
target_link_libraries(triangulation
        ${catkin_LIBRARIES}
        )

target_link_libraries(pose_estimation_3d2d
        ${catkin_LIBRARIES}
        Sophus
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}

        )

target_link_libraries(pose_estimation_3d3d
        ${catkin_LIBRARIES}
        Sophus
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}

        )

