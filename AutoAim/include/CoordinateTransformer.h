#ifndef COORDINATE_TRANSFORMER_H
#define COORDINATE_TRANSFORMER_H

#include <map>
#include <string>
#include "Pose.h"

class CoordinateTransformer {
private:
    inline static std::map<std::string, Pose> transforms = {
        {"Camera_to_Gimbal", Pose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        {"Gimbal_to_Odom", Pose(0.0, 0.0, 0.0, 0.1, 0.1, 0.1)}
    };

public:
    Pose transformToTarget(const Pose& camera_pose, const std::string& target_frame);
};

#endif