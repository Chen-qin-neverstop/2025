#include "ArmorTracker.h"
#include <chrono>

ArmorTracker::ArmorTracker(const cv::Point3f& initial_position, double dt) 
    : last_dt_(dt) {
    
    // 使用系统时间初始化
    last_update_time_ = cv::getTickCount() / cv::getTickFrequency();
    
    // 初始化卡尔曼滤波器
    kf_ = std::make_unique<KalmanFilter>(6, 3); // 6状态(x,y,z,vx,vy,vz), 3测量(x,y,z)
    
    cv::Mat state = (cv::Mat_<float>(6,1) << 
        initial_position.x, initial_position.y, initial_position.z, 0, 0, 0;
    kf_->init(state);
    
    // 设置初始转移矩阵
    updateTransitionMatrix(dt);
    
    // 设置过程噪声
    float noise = 0.1f * dt * dt;
    cv::Mat Q = (cv::Mat_<float>(6,6) << 
        noise,0,0,0,0,0,
        0,noise,0,0,0,0,
        0,0,noise,0,0,0,
        0,0,0,noise,0,0,
        0,0,0,0,noise,0,
        0,0,0,0,0,noise;
    kf_->setProcessNoiseCov(Q);
}

void ArmorTracker::update(const cv::Point3f& new_position, double dt) {
    // 更新时间戳
    last_update_time_ = cv::getTickCount() / cv::getTickFrequency();
    
    // 更新转移矩阵
    if (dt != last_dt_) {
        updateTransitionMatrix(dt);
        last_dt_ = dt;
    }
    
    // 更新滤波器
    cv::Mat measurement = (cv::Mat_<float>(3,1) << 
        new_position.x, new_position.y, new_position.z;
    kf_->correct(measurement);
    
    resetLostCount();
}

cv::Point3f ArmorTracker::predictNextPosition() const {
    cv::Mat prediction = kf_->predict();
    return cv::Point3f(
        prediction.at<float>(0),
        prediction.at<float>(1),
        prediction.at<float>(2)
    );
}

void ArmorTracker::updateTransitionMatrix(double dt) {
    cv::Mat F = (cv::Mat_<float>(6,6) << 
        1,0,0,dt,0,0,
        0,1,0,0,dt,0,
        0,0,1,0,0,dt,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1);
    kf_->setTransitionMatrix(F);
}