#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(int state_dim, int meas_dim, int ctrl_dim) 
    : state_dim_(state_dim), meas_dim_(meas_dim), ctrl_dim_(ctrl_dim),
      kf_(state_dim, meas_dim, ctrl_dim), initialized_(false) {
    
    // 初始化转移矩阵 (单位矩阵)
    cv::setIdentity(kf_.transitionMatrix);
    
    // 初始化测量矩阵 (只能观测位置)
    kf_.measurementMatrix = cv::Mat::zeros(meas_dim, state_dim, CV_32F);
    for(int i = 0; i < meas_dim; i++) {
        kf_.measurementMatrix.at<float>(i,i) = 1.0f;
    }
    
    // 初始化过程噪声协方差
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-4));
    
    // 初始化测量噪声协方差
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-2));
    
    // 初始化后验误差协方差
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(0.1));
}

void KalmanFilter::init(const cv::Mat& initial_state) {
    if(initial_state.rows != state_dim_ || initial_state.cols != 1) {
        throw std::runtime_error("Invalid initial state dimension");
    }
    
    // 初始化状态
    kf_.statePost = initial_state.clone();
    initialized_ = true;
}

cv::Mat KalmanFilter::predict(const cv::Mat& control) {
    if(!initialized_) {
        throw std::runtime_error("Kalman filter not initialized");
    }
    
    // 设置控制输入
    cv::Mat ctrl = control;
    if(ctrl.empty() && ctrl_dim_ > 0) {
        ctrl = cv::Mat::zeros(ctrl_dim_, 1, CV_32F);
    }
    
    return kf_.predict(ctrl);
}

cv::Mat KalmanFilter::correct(const cv::Mat& measurement) {
    if(!initialized_) {
        throw std::runtime_error("Kalman filter not initialized");
    }
    
    if(measurement.rows != meas_dim_ || measurement.cols != 1) {
        throw std::runtime_error("Invalid measurement dimension");
    }
    
    return kf_.correct(measurement);
}

void KalmanFilter::setTransitionMatrix(const cv::Mat& F) {
    if(F.rows != state_dim_ || F.cols != state_dim_) {
        throw std::runtime_error("Invalid transition matrix dimension");
    }
    F.copyTo(kf_.transitionMatrix);
}

void KalmanFilter::setMeasurementMatrix(const cv::Mat& H) {
    if(H.rows != meas_dim_ || H.cols != state_dim_) {
        throw std::runtime_error("Invalid measurement matrix dimension");
    }
    H.copyTo(kf_.measurementMatrix);
}

void KalmanFilter::setProcessNoiseCov(const cv::Mat& Q) {
    if(Q.rows != state_dim_ || Q.cols != state_dim_) {
        throw std::runtime_error("Invalid process noise covariance dimension");
    }
    Q.copyTo(kf_.processNoiseCov);
}

void KalmanFilter::setMeasurementNoiseCov(const cv::Mat& R) {
    if(R.rows != meas_dim_ || R.cols != meas_dim_) {
        throw std::runtime_error("Invalid measurement noise covariance dimension");
    }
    R.copyTo(kf_.measurementNoiseCov);
}

void KalmanFilter::setErrorCovPost(const cv::Mat& P) {
    if(P.rows != state_dim_ || P.cols != state_dim_) {
        throw std::runtime_error("Invalid error covariance dimension");
    }
    P.copyTo(kf_.errorCovPost);
}