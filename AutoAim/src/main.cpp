// 之后的输入需要修改为视频流的坐标系
#include "CoordinateTransformer.h"
#include <iostream>
#include <iomanip>
#include <vector>

std::vector<double> transformCoordinates(
    double x, double y, double z,
    double yaw, double pitch, double roll,
    const std::string& target_frame) 
{
    Pose camera_pose(x, y, z, roll, pitch, yaw);
    CoordinateTransformer transformer;
    Pose target_pose = transformer.transformToTarget(camera_pose, target_frame);
    
    auto pos = target_pose.getPosition();
    auto orient = target_pose.getOrientation();
    return {pos[0], pos[1], pos[2], orient[0], orient[1], orient[2]};
}

int main() {
    try {
        // 测试转换到Gimbal系
        auto result_gimbal = transformCoordinates(
            1.0, 2.0, 3.0,     // Camera系下的位置
            0.1, 0.1, 0.1,      // Camera系下的旋转 (roll,pitch,yaw)
            "/Gimbal"           // 目标坐标系
        );
        std::cout << "Camera→Gimbal: ";
        std::cout << result_gimbal[0] << " " << result_gimbal[1] << " " << result_gimbal[2] << " "
                  << result_gimbal[3] << " " << result_gimbal[4] << " " << result_gimbal[5] << std::endl;

        // 测试转换到Odom系
        auto result_odom = transformCoordinates(
            1.0, 2.0, 3.0,
            0.1, 0.1, 0.1,
            "/Odom"
        );
        std::cout << "Camera→Odom: ";
        std::cout << result_odom[0] << " " << result_odom[1] << " " << result_odom[2] << " "
                  << result_odom[3] << " " << result_odom[4] << " " << result_odom[5] << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}