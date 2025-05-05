#ifndef QUATERNION_H
#define QUATERNION_H

#include <vector>
#include <cmath>

class Quaternion {
private:
    double w, x, y, z;

public:
    Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0);
    static Quaternion fromEulerAngles(double roll, double pitch, double yaw);
    
    Quaternion normalized() const;
    Quaternion conjugate() const;
    Quaternion operator*(const Quaternion& q) const;
    
    std::vector<double> toEulerAngles() const;
    
    double getW() const;
    double getX() const;
    double getY() const;
    double getZ() const;
};

#endif