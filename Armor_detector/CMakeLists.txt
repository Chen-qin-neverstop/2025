cmake_minimum_required(VERSION 3.16)
project(MyOpenCVProject)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)

# 找到OpenCV包
find_package(OpenCV REQUIRED)
# find_package(OpenCV 4.8.0 REQUIRED PATHS ~/apps/opencv) 可以指定路径，opencv安装路径

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加一个可执行文件
add_executable(MyOpenCVProject main.cpp)

# 链接OpenCV库
target_link_libraries(MyOpenCVProject ${OpenCV_LIBS})


