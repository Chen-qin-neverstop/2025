#include "ArmorTracker.h"
#include <chrono>

ArmorTracker::ArmorTracker()
    : last_timestamp_(0.0)
    , velocity_(0, 0, 0)
    , last_position_(0, 0, 0)
    , predicted_position_(0, 0, 0) {
}

ArmorTracker::~ArmorTracker() {
}

cv::Point3f ArmorTracker::update(const cv::Point3f& position, double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (last_timestamp_ > 0) {
        // 计算时间差
        double dt = timestamp - last_timestamp_;
        if (dt > 0) {
            // 更新速度估计
            cv::Point3f new_velocity = (position - last_position_) / dt;
            // 使用指数移动平均来平滑速度
            velocity_ = 0.7f * velocity_ + 0.3f * new_velocity;
            
            // 预测下一个位置
            predicted_position_ = position + velocity_ * dt;
        }
    }
    
    // 更新状态
    last_position_ = position;
    last_timestamp_ = timestamp;
    position_history_.push_back(position);
    
    // 保持历史记录在合理的大小
    if (position_history_.size() > 30) {  // 保持约1秒的历史记录
        position_history_.erase(position_history_.begin());
    }
    
    return predicted_position_;
}

cv::Point3f ArmorTracker::getLastPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_position_;
}

double ArmorTracker::getLastTimestamp() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_timestamp_;
}

cv::Point3f ArmorTracker::getPredictedPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return predicted_position_;
}