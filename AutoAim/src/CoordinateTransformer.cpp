 #include "CoordinateTransformer.h"
 #include <stdexcept>

// 坐标系转换
Pose CoordinateTransformer::transformToTarget(const Pose& camera_pose, 
    const std::string& target_frame) {
// 检查目标坐标系是否支持
if (target_frame != "/Odom" && target_frame != "/Gimbal") {
throw std::runtime_error("Unsupported target frame: " + target_frame);
}

// 1. Camera → Gimbal（应用逆变换：+2m X方向） 纯平移 // 这里的逆变换不一定正确
Pose gimbal_pose(
camera_pose.getPosition()[0] + 2.0,  // X方向+2m
camera_pose.getPosition()[1],      
camera_pose.getPosition()[2],
camera_pose.getOrientation()[0],     // 旋转不变
camera_pose.getOrientation()[1],
camera_pose.getOrientation()[2]
);

// 2. 根据目标坐标系返回结果
if (target_frame == "/Gimbal") {
return gimbal_pose;  // 直接返回Gimbal系下的位姿
}
else if (target_frame == "/Odom") {
// Gimbal → Odom（应用旋转） 纯旋转
Quaternion q_rot = Quaternion::fromEulerAngles(0.1, 0.1, 0.1);
Quaternion q_result = q_rot * gimbal_pose.getQuaternion();
return Pose(gimbal_pose.getPosition(), q_result);
}

// 理论上不会执行到此处（因前面已检查）
throw std::runtime_error("Unreachable code");
}

