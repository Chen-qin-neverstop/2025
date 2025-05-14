#include "MotionEstimator.h"

MotionEstimator::MotionEstimator(int buffer_size, float fps) 
    : buffer_size_(buffer_size), fps_(fps) {}

MotionEstimator::MotionState MotionEstimator::update(const cv::Mat& rvec, const cv::Mat& tvec) {
    rvec_history_.push_back(rvec.clone());
    tvec_history_.push_back(tvec.clone());
    
    if (rvec_history_.size() > buffer_size_) {
        rvec_history_.pop_front();
        tvec_history_.pop_front();
    }
    
    MotionState state;
    
    if (rvec_history_.size() >= 2) {
        float dt = 1.0f / fps_;
            
        state.angular_velocity = calculateAngularVelocity(
            rvec_history_[rvec_history_.size()-2], 
            rvec_history_.back(), 
            dt);
            
        state.linear_velocity = calculateLinearVelocity(
            tvec_history_[tvec_history_.size()-2],
            tvec_history_.back(),
            dt);
            
        state.rotation_radius = calculateRotationRadius(
            state.linear_velocity,
            state.angular_velocity);
    }
    
    return state;
}

cv::Point3f MotionEstimator::calculateAngularVelocity(const cv::Mat& rvec1, const cv::Mat& rvec2, float dt) {
    cv::Mat rot_mat1, rot_mat2;
    cv::Rodrigues(rvec1, rot_mat1);
    cv::Rodrigues(rvec2, rot_mat2);
    
    cv::Mat delta_rot_mat = rot_mat2 * rot_mat1.t();
    
    cv::Mat delta_rvec;
    cv::Rodrigues(delta_rot_mat, delta_rvec);
    
    return cv::Point3f(
        delta_rvec.at<double>(0) / dt,
        delta_rvec.at<double>(1) / dt,
        delta_rvec.at<double>(2) / dt);
}

cv::Point3f MotionEstimator::calculateLinearVelocity(const cv::Mat& tvec1, const cv::Mat& tvec2, float dt) {
    return cv::Point3f(
        (tvec2.at<double>(0) - tvec1.at<double>(0)) / dt,
        (tvec2.at<double>(1) - tvec1.at<double>(1)) / dt,
        (tvec2.at<double>(2) - tvec1.at<double>(2)) / dt);
}

float MotionEstimator::calculateRotationRadius(const cv::Point3f& linear_vel, const cv::Point3f& angular_vel) {
    float angular_speed = cv::norm(angular_vel);
    if (angular_speed < 0.01f) return 0.0f;
    
    float tangential_speed = cv::norm(linear_vel);
    return tangential_speed / angular_speed;
}