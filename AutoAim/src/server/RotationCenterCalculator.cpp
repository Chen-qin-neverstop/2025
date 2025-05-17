#include "RotationCenterCalculator.h"
#include <cmath>

cv::Point3f RotationCenterCalculator::Calculate(
    const Pose& armor_pose,
    const cv::Point3f& linear_velocity,
    const cv::Point3f& angular_velocity) {
    
    // 获取位姿数据
    const std::vector<double>& pos = armor_pose.getPosition();
    const std::vector<double>& orient = armor_pose.getOrientation();
    
    // 转换为Point3f
    const cv::Point3f armor_position(pos[0], pos[1], pos[2]);
    const float yaw = orient[2];   // 注意顺序：orientation是roll,pitch,yaw
    const float pitch = orient[1];
    const float roll = orient[0];

    // 计算角速度大小
    const float angular_speed = cv::norm(angular_velocity);
    
    // 角速度过小时直接返回装甲板位置
    if (angular_speed < MIN_ANGULAR_SPEED) {
        return armor_position;
    }

    // 计算装甲板朝向向量（基于欧拉角）
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);

    // 计算旋转矩阵的Z轴方向（装甲板正前方）
    const cv::Point3f armor_direction(
        cy * cp,
        sy * cp,
        -sp
    );

    // 核心运动学公式：(v × ω)/|ω|²
    const cv::Point3f radius_vector = linear_velocity.cross(angular_velocity) / 
                                    (angular_speed * angular_speed);

    // 最终旋转中心 = 装甲板位置 + 半径矢量
    return armor_position + radius_vector;
}