cmake_minimum_required(VERSION 3.10)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加坐标转换器的可执行文件
add_executable(coord_transform
    main.cpp
    CoordinateTransformer.cpp
    Pose.cpp
    Quaternion.cpp
)

# 链接必要的库
target_link_libraries(coord_transform
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
)

# 设置包含目录
target_include_directories(coord_transform PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
