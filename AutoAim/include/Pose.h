#ifndef POSE_H
#define POSE_H

#include <vector>
#include "Quaternion.h"

class Pose {
private:
    std::vector<double> position;    // x,y,z
    std::vector<double> orientation; // roll,pitch,yaw

public:
    Pose(double x=0, double y=0, double z=0, 
         double roll=0, double pitch=0, double yaw=0);
    Pose(const std::vector<double>& pos, const Quaternion& q);
    
    Pose transform(const Pose& transform) const;
    Quaternion getQuaternion() const;
    
    std::vector<double> getPosition() const;
    std::vector<double> getOrientation() const;
    void print() const;
};

#endif