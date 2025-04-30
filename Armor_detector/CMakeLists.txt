cmake_minimum_required(VERSION 3.16)
project(myOpenCVProject)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)

# 找到OpenCV包
find_package(OpenCV REQUIRED)
# find_package(OpenCV 4.8.0 REQUIRED PATHS ~/apps/opencv) 可以指定路径，opencv安装路径

# 手动指定Eigen库路径，如果需要的话
# set(EIGEN3_INCLUDE_DIR /path/to/eigen)
# 如果手动指定了路径，使用下面的方式查找Eigen库
# find_package(Eigen3 3.3 REQUIRED NO_MODULE PATHS ${EIGEN3_INCLUDE_DIR})
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加一个可执行文件
add_executable(myOpenCVProject main.cpp)

# 链接OpenCV库
target_link_libraries(myOpenCVProject ${OpenCV_LIBS})

# 链接Eigen库
target_link_libraries(myOpenCVProject Eigen3::Eigen)    