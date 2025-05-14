cmake_minimum_required(VERSION 3.10)
project(RMVision2025)

add_subdirectory(AutoAim)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 查找必要的包
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

# 安装规则
install(DIRECTORY ${CMAKE_SOURCE_DIR}/config/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/config
        FILES_MATCHING PATTERN "*.yaml"
        PATTERN "*.xml"
        PATTERN "*.json")

# 创建必要的目录
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/config)
file(COPY ${CMAKE_SOURCE_DIR}/config/ DESTINATION ${CMAKE_BINARY_DIR}/config) 