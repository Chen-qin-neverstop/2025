#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <opencv2/opencv.hpp>

/**
 * @brief 专用于3D位置预测的卡尔曼滤波器
 * 
 * 状态向量: [x, y, z, vx, vy, vz]ᵀ
 * 测量向量: [x, y, z]ᵀ
 */
class RotationCenterKalmanFilter {
public:
    RotationCenterKalmanFilter();
    
    /**
     * @brief 初始化/重置滤波器
     * @param init_pos 初始位置 (x,y,z)
     */
    void init(const cv::Point3f& init_pos);
    
    /**
     * @brief 更新滤波器状态并返回预测值
     * @param measurement 测量到的位置
     * @param dt 时间步长(秒)。若<=0则自动计算
     * @return 预测的下一时刻位置
     */
    cv::Point3f update(const cv::Point3f& measurement, float dt = -1.0f);
    
    /**
     * @brief 仅进行预测
     * @param dt 预测的时间步长(秒)
     * @return 预测位置 
     */
    cv::Point3f predict(float dt = 0.033f); // 默认1/30秒

private:
    cv::KalmanFilter kf_impl_;
    double last_timestamp_;
    bool is_initialized_;
};

#endif