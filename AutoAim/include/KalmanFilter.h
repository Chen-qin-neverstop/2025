#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <opencv2/opencv.hpp>

class KalmanFilter {
public:
    KalmanFilter(int state_dim = 6, int meas_dim = 3, int ctrl_dim = 0);
    
    void init(const cv::Mat& initial_state);
    cv::Mat predict(const cv::Mat& control = cv::Mat());
    cv::Mat correct(const cv::Mat& measurement);
    
    void setTransitionMatrix(const cv::Mat& F);
    void setMeasurementMatrix(const cv::Mat& H);
    void setProcessNoiseCov(const cv::Mat& Q);
    void setMeasurementNoiseCov(const cv::Mat& R);
    void setErrorCovPost(const cv::Mat& P);
    cv::Mat getStatePost() const { return kf_.statePost; }
    cv::Mat getErrorCovPost() const { return kf_.errorCovPost; }
     

private:
    int state_dim_;     // 状态维度 (x,y,z,vx,vy,vz)
    int meas_dim_;      // 测量维度 (x,y,z)
    int ctrl_dim_;      // 控制输入维度
    
    cv::KalmanFilter kf_;
    bool initialized_;
};

#endif // KALMAN_FILTER_H