#include "CoordinateTransformer.h"
#include <stdexcept>

Pose CoordinateTransformer::transformToTarget(const Pose& camera_pose, 
    const std::string& target_frame) {
    
    if (target_frame != "/Odom" && target_frame != "/Gimbal") {
        throw std::runtime_error("Unsupported target frame: " + target_frame);
    }

    // 1. Camera → Gimbal（应用逆变换：+2m X方向）
    Pose gimbal_pose(
        camera_pose.getPosition()[0] + 2.0,
        camera_pose.getPosition()[1],      
        camera_pose.getPosition()[2],
        camera_pose.getOrientation()[0],
        camera_pose.getOrientation()[1],
        camera_pose.getOrientation()[2]
    );

    if (target_frame == "/Gimbal") {
        return gimbal_pose;
    }
    else if (target_frame == "/Odom") {
        // Gimbal → Odom（应用旋转）
        Quaternion q_rot = Quaternion::fromEulerAngles(0.1, 0.1, 0.1);
        Quaternion q_result = q_rot * gimbal_pose.getQuaternion();
        return Pose(gimbal_pose.getPosition(), q_result);
    }

    throw std::runtime_error("Unreachable code");
}