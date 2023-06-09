#声明要求的cmake最低版本
cmake_minimum_required( VERSION 2.8)

#声明一个cmake工程
project(HelloSLAM)

#添加一个可执行文件
add_executable(helloSLAM helloSLAM.cpp)

#静态库每次被调用都会生成一个副本，而共享库则只有一个副本，更省空间
#将文件libHelloSLAM.cpp编译成一个叫hello的静态库
add_library(hello libHelloSLAM.cpp)

#将文件libHelloSLAM.cpp编译成动态库
add_library(hello_shared SHARED libHelloSLAM.cpp)


#通过这两行代码，此时useHello可以顺利使用hello_shared库中的代码
add_executable(useHello useHello.cpp)
target_link_libraries(useHello hello_shared)