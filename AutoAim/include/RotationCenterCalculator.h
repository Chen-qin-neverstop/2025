#ifndef ROTATION_CENTER_CALCULATOR_H
#define ROTATION_CENTER_CALCULATOR_H

#include <opencv2/core/core.hpp>
#include "Pose.h"

class RotationCenterCalculator {
public:
    /**
     * @brief 计算旋转中心的世界坐标
     * @param armor_pose 装甲板位姿（通过getPosition()和getOrientation()获取）
     * @param linear_velocity 线速度(m/s)
     * @param angular_velocity 角速度(rad/s)
     * @return 旋转中心的世界坐标系3D位置
     */
    static cv::Point3f Calculate(
        const Pose& armor_pose,
        const cv::Point3f& linear_velocity,
        const cv::Point3f& angular_velocity);

private:
    static constexpr float MIN_ANGULAR_SPEED = 0.0001f; // 有效角速度阈值(rad/s)
};

#endif