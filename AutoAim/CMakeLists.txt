cmake_minimum_required(VERSION 3.10)
project(ArmorDetectionSystem)

set(CMAKE_CXX_STANDARD 11)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 服务器可执行文件
add_executable(armor_server
    src/server/server.cpp
    src/server/ImageProcess.cpp
    src/server/CoordinateTransformer.cpp
    src/server/MotionEstimator.cpp
    src/server/RotationCenterCalculator.cpp
    src/server/Pose.cpp
    src/server/Quaternion.cpp
    src/server/Protocol.cpp
)

target_link_libraries(armor_server ${OpenCV_LIBS})

# 客户端可执行文件
add_executable(armor_client
    src/client/client.cpp
    src/common/utils.cpp
    src/server/Protocol.cpp
)

target_link_libraries(armor_client ${OpenCV_LIBS})

# 测试可执行文件
add_executable(test_pose test/test_pose.cpp src/server/Pose.cpp src/server/Quaternion.cpp)
add_executable(test_protocol test/test_protocol.cpp src/server/Protocol.cpp)