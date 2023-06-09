cmake_minimum_required(VERSION 3.19)
project(SlamBook_zxj)

set(CMAKE_CXX_STANDARD 14)

include_directories(ch2)

add_executable(SlamBook_zxj
        ch2/helloSLAM.cpp
        ch2/libHelloSLAM.cpp
        ch2/libHelloSLAM.h
        ch2/useHello.cpp
        ch3/useEigen/eigenMatrix.cpp)

add_executable(useSophus useSophus.cpp)
