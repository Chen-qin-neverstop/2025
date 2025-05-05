#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <deque>
#include <cmath>

class MotionEstimator {
public:
    struct MotionState {
        cv::Point3f linear_velocity;    // 线速度 (m/s)
        cv::Point3f angular_velocity;   // 角速度 (rad/s)
        float rotation_radius;          // 旋转半径 (m)
    };

    MotionEstimator(int buffer_size = 5, float fps = 30.0f) 
        : buffer_size_(buffer_size), fps_(fps) {}

    // 更新运动状态估计
    MotionState update(const cv::Mat& rvec, const cv::Mat& tvec);

private:
    int buffer_size_;                   // 用于平滑的历史数据量
    float fps_;                         // 帧率
    std::deque<cv::Mat> rvec_history_;  // 旋转向量历史
    std::deque<cv::Mat> tvec_history_;  // 平移向量历史

    // 计算角速度
    cv::Point3f calculateAngularVelocity(const cv::Mat& rvec1, const cv::Mat& rvec2, float dt);

    // 计算线速度
    cv::Point3f calculateLinearVelocity(const cv::Mat& tvec1, const cv::Mat& tvec2, float dt);

    // 计算旋转半径
    float calculateRotationRadius(const cv::Point3f& linear_vel, const cv::Point3f& angular_vel);
};

#endif // MOTION_ESTIMATOR_H