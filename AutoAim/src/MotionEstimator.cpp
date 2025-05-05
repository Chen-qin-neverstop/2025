#include "MotionEstimator.h"
#include <iostream>

using namespace cv;
using namespace std;

MotionEstimator::MotionState MotionEstimator::update(const Mat& rvec, const Mat& tvec) {
    // 添加到历史数据
    rvec_history_.push_back(rvec.clone());
    tvec_history_.push_back(tvec.clone());
    
    // 保持缓冲区大小
    if (rvec_history_.size() > buffer_size_) {
        rvec_history_.pop_front();
        tvec_history_.pop_front();
    }
    
    MotionState state;
    
    // 需要至少2帧数据才能计算速度
    if (rvec_history_.size() >= 2) {
        float dt = 1.0f / fps_;  // 时间间隔
        
        // 计算角速度和线速度
        state.angular_velocity = calculateAngularVelocity(
            rvec_history_[rvec_history_.size()-2], 
            rvec_history_.back(), 
            dt);
            
        state.linear_velocity = calculateLinearVelocity(
            tvec_history_[tvec_history_.size()-2],
            tvec_history_.back(),
            dt);
            
        // 计算旋转半径
        state.rotation_radius = calculateRotationRadius(
            state.linear_velocity,
            state.angular_velocity);
    }
    
    return state;
}

Point3f MotionEstimator::calculateAngularVelocity(const Mat& rvec1, const Mat& rvec2, float dt) {
    // 将旋转向量转换为旋转矩阵
    Mat rot_mat1, rot_mat2;
    Rodrigues(rvec1, rot_mat1);
    Rodrigues(rvec2, rot_mat2);
    
    // 计算相对旋转矩阵
    Mat delta_rot_mat = rot_mat2 * rot_mat1.t();
    
    // 将相对旋转矩阵转换为旋转向量
    Mat delta_rvec;
    Rodrigues(delta_rot_mat, delta_rvec);
    
    // 计算角速度 (rad/s)
    return Point3f(
        delta_rvec.at<double>(0) / dt,
        delta_rvec.at<double>(1) / dt,
        delta_rvec.at<double>(2) / dt);
}

Point3f MotionEstimator::calculateLinearVelocity(const Mat& tvec1, const Mat& tvec2, float dt) {
    // 计算线速度 (m/s)
    return Point3f(
        (tvec2.at<double>(0) - tvec1.at<double>(0)) / dt,
        (tvec2.at<double>(1) - tvec1.at<double>(1)) / dt,
        (tvec2.at<double>(2) - tvec1.at<double>(2)) / dt);
}

float MotionEstimator::calculateRotationRadius(const Point3f& linear_vel, const Point3f& angular_vel) {
    // 计算角速度大小
    float angular_speed = norm(angular_vel);
    
    // 如果角速度太小，无法准确计算旋转半径
    if (angular_speed < 0.01f) {
        return 0.0f;
    }
    
    // 计算切向速度大小
    float tangential_speed = norm(linear_vel);
    
    // 旋转半径 = 切向速度 / 角速度
    return tangential_speed / angular_speed;
}