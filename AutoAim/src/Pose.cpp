#include "Pose.h"
#include <iostream>
#include <iomanip>
// 定义 Pose 类的构造函数和成员函数  输入顺序依旧是
Pose::Pose(double x, double y, double z, 
           double roll, double pitch, double yaw)
    : position({x, y, z}),
      orientation({roll, pitch, yaw}) {}

Pose::Pose(const std::vector<double>& pos, const Quaternion& q) 
    : position(pos) {
    auto euler = q.toEulerAngles();  // 获取 [roll,pitch,yaw]
    orientation = {euler[0], euler[1], euler[2]}; // 转为 [roll,pitch,yaw]
}

Quaternion Pose::getQuaternion() const {
    return Quaternion::fromEulerAngles(
        orientation[0],  // roll
        orientation[1],  // pitch
        orientation[2]   // yaw
    );
}

std::vector<double> Pose::getPosition() const { 
    return position; 
}

std::vector<double> Pose::getOrientation() const { 
    return orientation; 
}

Pose Pose::transform(const Pose& tf) const {
    // 1. 获取四元数表示（注意顺序：roll, pitch, yaw）
    Quaternion q_this = this->getQuaternion();
    Quaternion q_tf = tf.getQuaternion();
    
    // 2. 组合旋转（顺序必须为 q_tf * q_this）
    Quaternion q_result = q_tf * q_this;
    
    // 3. 变换位置（先旋转后平移）
    Quaternion p_quat(0.0, position[0], position[1], position[2]);
    Quaternion p_rotated = q_tf * p_quat * q_tf.conjugate();
    
    // 4. 应用平移
    std::vector<double> tf_pos = tf.getPosition();
    std::vector<double> new_pos = {
        tf_pos[0] + p_rotated.getX(),
        tf_pos[1] + p_rotated.getY(),
        tf_pos[2] + p_rotated.getZ()
    };
    
    // 5. 返回新位姿
    return Pose(new_pos, q_result);
}

void Pose::print() const {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << position[0] << " " << position[1] << " " << position[2] << " "
              << orientation[0] << " " << orientation[1] << " " << orientation[2];
}