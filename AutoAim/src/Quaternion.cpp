#include "Quaternion.h"
#include <cmath>
#include <stdexcept>
// 四元数类的实现
Quaternion::Quaternion(double w, double x, double y, double z) 
    : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::fromEulerAngles(double roll, double pitch, double yaw) {
    // 将欧拉角转换为四元数
    // 注意：这里的 roll, pitch, yaw 是按照 (roll, pitch, yaw) 的顺序,但是旋转顺序是 (yaw, pitch, roll)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    return Quaternion(    // ZYX 顺序四元数构造公式（即 yaw → pitch → roll）
        cy * cp * cr + sy * sp * sr,   // W
        sy * cp * cr - cy * sp * sr,   // X
        cy * sp * cr + sy * cp * sr,   // Y
        cy * cp * sr - sy * sp * cr    // Z
    );
}

Quaternion Quaternion::normalized() const {
    double norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm < 1e-12) throw std::runtime_error("Cannot normalize zero quaternion");
    return Quaternion(w/norm, x/norm, y/norm, z/norm);
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w*q.w - x*q.x - y*q.y - z*q.z,
        w*q.x + x*q.w + y*q.z - z*q.y,
        w*q.y - x*q.z + y*q.w + z*q.x,
        w*q.z + x*q.y - y*q.x + z*q.w
    );
}

std::vector<double> Quaternion::toEulerAngles() const {
    std::vector<double> angles(3);
    
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);
    
    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI_2, sinp);
    else
        angles[1] = std::asin(sinp);
    
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    
    return angles;
}

double Quaternion::getW() const { return w; }
double Quaternion::getX() const { return x; }
double Quaternion::getY() const { return y; }
double Quaternion::getZ() const { return z; }