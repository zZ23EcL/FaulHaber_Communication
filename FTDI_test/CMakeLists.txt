cmake_minimum_required(VERSION 3.22)
project(FTDI_test)

set(CMAKE_CXX_STANDARD 14)

# 头文件路径
set(INC_DIR /usr/local/include)
# 库文件路径
set(LIB_DIR /usr/local/lib)

include_directories(${INC_DIR})
link_directories(${LIB_DIR})
link_libraries(ftd2xx)

add_executable(FTDI_test main.cpp RS232.cpp RS232.h MiniPID.h MiniPID.cpp)
find_package(Threads REQUIRED)
# 链接 FTDI 库
target_link_libraries(FTDI_test ftd2xx Threads::Threads)
