cmake_minimum_required(VERSION 3.10)
project(CoordinateTransform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加可执行文件
add_executable(coord_transform
    src/main.cpp
    src/Quaternion.cpp
    src/Pose.cpp
    src/CoordinateTransformer.cpp
)

# 包含头文件目录
target_include_directories(coord_transform PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 链接数学库
target_link_libraries(coord_transform m)