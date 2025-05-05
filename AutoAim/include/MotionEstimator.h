#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <deque>
#include <cmath>

class MotionEstimator {
public:
    struct MotionState {
        cv::Point3f linear_velocity;
        cv::Point3f angular_velocity;
        float rotation_radius;
    };

    MotionEstimator(int buffer_size = 5, float fps = 30.0f);
    MotionState update(const cv::Mat& rvec, const cv::Mat& tvec);

private:
    int buffer_size_;
    float fps_;
    std::deque<cv::Mat> rvec_history_;
    std::deque<cv::Mat> tvec_history_;

    cv::Point3f calculateAngularVelocity(const cv::Mat& rvec1, const cv::Mat& rvec2, float dt);
    cv::Point3f calculateLinearVelocity(const cv::Mat& tvec1, const cv::Mat& tvec2, float dt);
    float calculateRotationRadius(const cv::Point3f& linear_vel, const cv::Point3f& angular_vel);
};

#endif