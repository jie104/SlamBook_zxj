#声明最低要求的cmake版本
cmake_minimum_required(VERSION 2.8)

#声明一个cmake工程
project(EigenMatrix)

#添加头文件
include_directories("/usr/include/eigen3")

#添加一个可执行文件
add_executable(useEigen eigenMatrix.cpp)

