cmake_minimum_required(VERSION 3.10)
project(AutoAim)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# 添加头文件目录
include_directories(${OpenCV_INCLUDE_DIRS} include)

# 服务器源文件
set(SERVER_SOURCES
    src/server/ArmorDetectionServer.cpp
    src/server/Protocol.cpp
    src/server/ImageProcess.cpp
    src/server/ArmorTracker.cpp
    src/server/MotionEstimator.cpp
    src/server/RotationCenterCalculator.cpp
)

# 客户端源文件
set(CLIENT_SOURCES
    src/client/VideoClient.cpp
)

# 添加可执行文件
add_executable(armor_detection_server ${SERVER_SOURCES})
add_executable(video_client ${CLIENT_SOURCES})

# 链接库
target_link_libraries(armor_detection_server ${OpenCV_LIBS} Threads::Threads)
target_link_libraries(video_client ${OpenCV_LIBS} Threads::Threads)

# 安装目标
install(TARGETS armor_detection_server video_client
    RUNTIME DESTINATION bin
)