cmake_minimum_required(VERSION 2.8)
project(ch6)

set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
        OpenCV
        Ceres
#        G2O
#        Eigen
        )


list(APPEND CMAKE_MODULE_PATH  /home/zxj/lib/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)
find_package( G2O REQUIRED )
include_directories(
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3" )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gaussNewton gaussNewton.cpp)
add_executable(ceresCurveFitting ceresCurveFitting.cpp)

add_executable(g2oCurveFitting g2oCurveFitting.cpp)

target_link_libraries(gaussNewton
        ${catkin_LIBRARIES}
        )

target_link_libraries(ceresCurveFitting
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        )

target_link_libraries(g2oCurveFitting
        ${catkin_LIBRARIES}
#        ${g2o_LIBRARIES}
#        g2o_types_sba
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}

        )
