#include "Pose.h"
#include <iostream>
#include <iomanip>

Pose::Pose(double x, double y, double z, 
           double roll, double pitch, double yaw)
    : position({x, y, z}),
      orientation({roll, pitch, yaw}) {}

Pose::Pose(const std::vector<double>& pos, const Quaternion& q) 
    : position(pos) {
    auto euler = q.toEulerAngles();
    orientation = {euler[0], euler[1], euler[2]};
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
    Quaternion q_this = this->getQuaternion();
    Quaternion q_tf = tf.getQuaternion();
    
    Quaternion q_result = q_tf * q_this;
    
    Quaternion p_quat(0.0, position[0], position[1], position[2]);
    Quaternion p_rotated = q_tf * p_quat * q_tf.conjugate();
    
    std::vector<double> tf_pos = tf.getPosition();
    std::vector<double> new_pos = {
        tf_pos[0] + p_rotated.getX(),
        tf_pos[1] + p_rotated.getY(),
        tf_pos[2] + p_rotated.getZ()
    };
    
    return Pose(new_pos, q_result);
}

void Pose::print() const {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << position[0] << " " << position[1] << " " << position[2] << " "
              << orientation[0] << " " << orientation[1] << " " << orientation[2];
}