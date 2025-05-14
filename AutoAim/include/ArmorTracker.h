#ifndef ARMOR_TRACKER_H
#define ARMOR_TRACKER_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <mutex>

class ArmorTracker {
public:
    ArmorTracker();
    ~ArmorTracker();

    // 更新跟踪器状态
    cv::Point3f update(const cv::Point3f& position, double timestamp);
    
    // 获取最后检测到的位置
    cv::Point3f getLastPosition() const;
    
    // 获取最后的时间戳
    double getLastTimestamp() const;

    // 获取预测的位置
    cv::Point3f getPredictedPosition() const;

private:
    std::vector<cv::Point3f> position_history_;
    cv::Point3f last_position_;
    cv::Point3f predicted_position_;
    double last_timestamp_;
    mutable std::mutex mutex_;
    
    // 简单的卡尔曼滤波器参数
    cv::Point3f velocity_;
    static constexpr double dt = 0.033;  // 假设30fps
    static constexpr double process_noise = 0.1;
    static constexpr double measurement_noise = 0.1;
};

#endif // ARMOR_TRACKER_H