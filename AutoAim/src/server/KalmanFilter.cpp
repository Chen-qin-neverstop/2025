// // #include "KalmanFilter.h"

// // KalmanFilter::KalmanFilter(int state_dim, int meas_dim, int ctrl_dim) 
// //     : state_dim_(state_dim), meas_dim_(meas_dim), ctrl_dim_(ctrl_dim),
// //       kf_(state_dim, meas_dim, ctrl_dim), initialized_(false) {
    
// //     // 初始化转移矩阵 (单位矩阵)
// //     cv::setIdentity(kf_.transitionMatrix);
    
// //     // 初始化测量矩阵 (只能观测位置)
// //     kf_.measurementMatrix = cv::Mat::zeros(meas_dim, state_dim, CV_32F);
// //     for(int i = 0; i < meas_dim; i++) {
// //         kf_.measurementMatrix.at<float>(i,i) = 1.0f;
// //     }
    
// //     // 初始化过程噪声协方差
// //     cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-4));
    
// //     // 初始化测量噪声协方差
// //     cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-2));
    
// //     // 初始化后验误差协方差
// //     cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(0.1));
// // }

// // void KalmanFilter::init(const cv::Mat& initial_state) {
// //     if(initial_state.rows != state_dim_ || initial_state.cols != 1) {
// //         throw std::runtime_error("Invalid initial state dimension");
// //     }
    
// //     // 初始化状态
// //     kf_.statePost = initial_state.clone();
// //     initialized_ = true;
// // }

// // cv::Mat KalmanFilter::predict(const cv::Mat& control) {
// //     if(!initialized_) {
// //         throw std::runtime_error("Kalman filter not initialized");
// //     }
    
// //     // 设置控制输入
// //     cv::Mat ctrl = control;
// //     if(ctrl.empty() && ctrl_dim_ > 0) {
// //         ctrl = cv::Mat::zeros(ctrl_dim_, 1, CV_32F);
// //     }
    
// //     return kf_.predict(ctrl);
// // }

// // cv::Mat KalmanFilter::correct(const cv::Mat& measurement) {
// //     if(!initialized_) {
// //         throw std::runtime_error("Kalman filter not initialized");
// //     }
    
// //     if(measurement.rows != meas_dim_ || measurement.cols != 1) {
// //         throw std::runtime_error("Invalid measurement dimension");
// //     }
    
// //     return kf_.correct(measurement);
// // }

// // void KalmanFilter::setTransitionMatrix(const cv::Mat& F) {
// //     if(F.rows != state_dim_ || F.cols != state_dim_) {
// //         throw std::runtime_error("Invalid transition matrix dimension");
// //     }
// //     F.copyTo(kf_.transitionMatrix);
// // }

// // void KalmanFilter::setMeasurementMatrix(const cv::Mat& H) {
// //     if(H.rows != meas_dim_ || H.cols != state_dim_) {
// //         throw std::runtime_error("Invalid measurement matrix dimension");
// //     }
// //     H.copyTo(kf_.measurementMatrix);
// // }

// // void KalmanFilter::setProcessNoiseCov(const cv::Mat& Q) {
// //     if(Q.rows != state_dim_ || Q.cols != state_dim_) {
// //         throw std::runtime_error("Invalid process noise covariance dimension");
// //     }
// //     Q.copyTo(kf_.processNoiseCov);
// // }

// // void KalmanFilter::setMeasurementNoiseCov(const cv::Mat& R) {
// //     if(R.rows != meas_dim_ || R.cols != meas_dim_) {
// //         throw std::runtime_error("Invalid measurement noise covariance dimension");
// //     }
// //     R.copyTo(kf_.measurementNoiseCov);
// // }

// // void KalmanFilter::setErrorCovPost(const cv::Mat& P) {
// //     if(P.rows != state_dim_ || P.cols != state_dim_) {
// //         throw std::runtime_error("Invalid error covariance dimension");
// //     }
// //     P.copyTo(kf_.errorCovPost);
// // }

// #include "KalmanFilter.h"
// #include <stdexcept>

// KalmanFilter::KalmanFilter(int state_dim, int meas_dim, int ctrl_dim) 
//     : state_dim_(state_dim), meas_dim_(meas_dim), ctrl_dim_(ctrl_dim),
//       kf_(state_dim, meas_dim, ctrl_dim), initialized_(false) {
    
//     // 初始化转移矩阵 (单位矩阵)
//     cv::setIdentity(kf_.transitionMatrix);
    
//     // 初始化测量矩阵 (只能观测位置)
//     kf_.measurementMatrix = cv::Mat::zeros(meas_dim, state_dim, CV_32F);
//     for(int i = 0; i < meas_dim; i++) {
//         kf_.measurementMatrix.at<float>(i,i) = 1.0f;
//     }
    
//     // 初始化过程噪声协方差
//     cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-4));
    
//     // 初始化测量噪声协方差
//     cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-2));
    
//     // 初始化后验误差协方差
//     cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(0.1));
// }

// void KalmanFilter::init(const cv::Mat& initial_state) {
//     if(initial_state.rows != state_dim_ || initial_state.cols != 1) {
//         throw std::runtime_error("Invalid initial state dimension");
//     }
    
//     // 初始化状态
//     kf_.statePost = initial_state.clone();
//     initialized_ = true;
// }

// cv::Mat KalmanFilter::predict(const cv::Mat& control) {
//     if(!initialized_) {
//         throw std::runtime_error("Kalman filter not initialized");
//     }
    
//     // 设置控制输入
//     cv::Mat ctrl = control;
//     if(ctrl.empty() && ctrl_dim_ > 0) {
//         ctrl = cv::Mat::zeros(ctrl_dim_, 1, CV_32F);
//     }
    
//     return kf_.predict(ctrl);
// }

// cv::Mat KalmanFilter::correct(const cv::Mat& measurement) {
//     if(!initialized_) {
//         throw std::runtime_error("Kalman filter not initialized");
//     }
    
//     if(measurement.rows != meas_dim_ || measurement.cols != 1) {
//         throw std::runtime_error("Invalid measurement dimension");
//     }
    
//     return kf_.correct(measurement);
// }

// void KalmanFilter::setTransitionMatrix(const cv::Mat& F) {
//     if(F.rows != state_dim_ || F.cols != state_dim_) {
//         throw std::runtime_error("Invalid transition matrix dimension");
//     }
//     F.copyTo(kf_.transitionMatrix);
// }

// void KalmanFilter::setMeasurementMatrix(const cv::Mat& H) {
//     if(H.rows != meas_dim_ || H.cols != state_dim_) {
//         throw std::runtime_error("Invalid measurement matrix dimension");
//     }
//     H.copyTo(kf_.measurementMatrix);
// }

// void KalmanFilter::setProcessNoiseCov(const cv::Mat& Q) {
//     if(Q.rows != state_dim_ || Q.cols != state_dim_) {
//         throw std::runtime_error("Invalid process noise covariance dimension");
//     }
//     Q.copyTo(kf_.processNoiseCov);
// }

// void KalmanFilter::setMeasurementNoiseCov(const cv::Mat& R) {
//     if(R.rows != meas_dim_ || R.cols != meas_dim_) {
//         throw std::runtime_error("Invalid measurement noise covariance dimension");
//     }
//     R.copyTo(kf_.measurementNoiseCov);
// }

// void KalmanFilter::setErrorCovPost(const cv::Mat& P) {
//     if(P.rows != state_dim_ || P.cols != state_dim_) {
//         throw std::runtime_error("Invalid error covariance dimension");
//     }
//     P.copyTo(kf_.errorCovPost);
// }


#include "KalmanFilter.h"
#include <stdexcept>

RotationCenterKalmanFilter::RotationCenterKalmanFilter() 
    : kf_impl_(6, 3, 0), // 6状态, 3测量, 0控制
      is_initialized_(false) {
    
    // 状态转移矩阵 (匀速模型)
    kf_impl_.transitionMatrix = (cv::Mat_<float>(6,6) << 
        1,0,0,1,0,0,
        0,1,0,0,1,0,
        0,0,1,0,0,1,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1);
    
    // 测量矩阵 (只能观测位置)
    kf_impl_.measurementMatrix = (cv::Mat_<float>(3,6) << 
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0);
    
    // 协方差矩阵初始化
    cv::setIdentity(kf_impl_.processNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(kf_impl_.measurementNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kf_impl_.errorCovPost, cv::Scalar::all(0.1));
}

void RotationCenterKalmanFilter::init(const cv::Point3f& init_pos) {
    kf_impl_.statePost = (cv::Mat_<float>(6,1) << 
        init_pos.x, init_pos.y, init_pos.z, 
        0, 0, 0); // 初始速度设为0
    last_timestamp_ = cv::getTickCount() / cv::getTickFrequency();
    is_initialized_ = true;
}

cv::Point3f RotationCenterKalmanFilter::update(const cv::Point3f& measurement, float dt) {
    if (!is_initialized_) {
        init(measurement);
        return measurement;
    }
    
    // 自动计算时间步长
    double current_time = cv::getTickCount() / cv::getTickFrequency();
    if (dt <= 0) {
        dt = current_time - last_timestamp_;
        dt = std::max(dt, 0.001f); // 防止dt为0
    }
    last_timestamp_ = current_time;
    
    // 更新转移矩阵中的时间项
    kf_impl_.transitionMatrix.at<float>(0,3) = dt;
    kf_impl_.transitionMatrix.at<float>(1,4) = dt;
    kf_impl_.transitionMatrix.at<float>(2,5) = dt;
    
    // 预测和更新
    cv::Mat prediction = kf_impl_.predict();
    cv::Mat meas = (cv::Mat_<float>(3,1) << measurement.x, measurement.y, measurement.z);
    kf_impl_.correct(meas);
    
    return cv::Point3f(prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2));
}

cv::Point3f RotationCenterKalmanFilter::predict(float dt) {
    if (!is_initialized_) {
        throw std::runtime_error("Filter not initialized");
    }
    
    kf_impl_.transitionMatrix.at<float>(0,3) = dt;
    kf_impl_.transitionMatrix.at<float>(1,4) = dt;
    kf_impl_.transitionMatrix.at<float>(2,5) = dt;
    
    cv::Mat prediction = kf_impl_.predict();
    return cv::Point3f(prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2));
}