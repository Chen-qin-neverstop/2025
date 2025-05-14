cmake_minimum_required(VERSION 3.10)
project(experiment_demo)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ../AutoAim/include
    ../Armor_detector
    ../CoordinateTransformer
)

add_executable(experiment_demo
    main.cpp
    ../AutoAim/src/server/ImageProcess.cpp
    ../AutoAim/src/server/KalmanFilter.cpp
    ../AutoAim/src/server/Pose.cpp
    ../AutoAim/src/server/Quaternion.cpp
)

target_link_libraries(experiment_demo
    ${OpenCV_LIBS}
) 