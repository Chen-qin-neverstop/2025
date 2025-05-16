#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
#include "KalmanFilter.h"
#include "ArmorTracker.h"

using namespace std;
using namespace cv;

double timestamp = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();

void drawRotationCenter(cv::Mat& frame, const cv::Point3f& center, 
                       const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    std::vector<cv::Point3f> points{center};
    std::vector<cv::Point2f> projected_points;
    
    cv::projectPoints(points, cv::Mat::zeros(3,1,CV_32F), cv::Mat::zeros(3,1,CV_32F),
                     camera_matrix, dist_coeffs, projected_points);
    
    if (!projected_points.empty()) {
        cv::circle(frame, projected_points[0], 10, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "RC: " + std::to_string(center.x) + "," + std::to_string(center.y), 
                   projected_points[0] + cv::Point2f(15,0), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
    }
}

// 初始化旋转中心卡尔曼滤波器
void initRotationCenterKF(::KalmanFilter& kf) {
    // 状态维度: 6 (x,y,z + vx,vy,vz)
    // 测量维度: 3 (x,y,z)
    kf = ::KalmanFilter(6, 3);
    // 状态转移矩阵 (匀速模型)
    cv::Mat F = cv::Mat::eye(6, 6, CV_32F);
    F.at<float>(0,3) = 1.0f;  // x + vx*dt
    F.at<float>(1,4) = 1.0f;  // y + vy*dt
    F.at<float>(2,5) = 1.0f;  // z + vz*dt
    kf.setTransitionMatrix(F);
    
    // 测量矩阵 (只能观测位置)
    cv::Mat H = cv::Mat::zeros(3, 6, CV_32F);
    H.at<float>(0,0) = 1.0f;
    H.at<float>(1,1) = 1.0f;
    H.at<float>(2,2) = 1.0f;
    kf.setMeasurementMatrix(H);
    
    // 过程噪声协方差
    cv::Mat Q = cv::Mat::eye(6, 6, CV_32F) * 1e-4;
    kf.setProcessNoiseCov(Q);
    
    // 测量噪声协方差
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F) * 1e-2;
    kf.setMeasurementNoiseCov(R);
    
    // 后验误差协方差
    cv::Mat P = cv::Mat::eye(6, 6, CV_32F) * 0.1;
    kf.setErrorCovPost(P);
}

int main(int argc, char** argv) {
// 相机参数
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
    2065.0580175762857, 0.0, 658.9098266395495,
    0.0, 2086.886458338243, 531.5333174739342,
    0.0, 0.0, 1.0);
// 畸变系数
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 
    -0.051836613762195866, 0.29341513924119095, 
    0.001501183796729562, 0.0009386915104617738, 0.0);
// 旋转半径需要根据目标实际尺寸设置（示例值，单位：米）
float rotation_radius = 0.5;

// 初始化跟踪器和滤波器
    ArmorTracker armor_tracker;
    ::KalmanFilter rotation_center_kf;
    initRotationCenterKF(rotation_center_kf);
    bool is_first_frame = true;
    double last_timestamp = 0.0; 

    // 改为读取视频
    std::string video_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/2.mp4"; // 修改为你的视频路径
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "无法打开视频: " << video_path << endl;
        return -1;
    }

    int frame_count = 0;
    Mat frame;
    
    MotionEstimator motion_estimator; // 在while循环之前声明
    // 在while循环之前声明
    cv::Point3f current_rotation_center;
    RotationCenterCalculator rotation_center_calculator;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cout << "视频处理完成" << endl;
            break;
        }
        
        frame_count++;
        // cout << "\n处理第 " << frame_count << " 帧" << endl;

        //原始图片处理代码注释掉
        // std::string image_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/21.jpg";
        // Mat img = imread(image_path);
        // if (img.empty()) {
        //     cerr << "无法读取图片: " << image_path << endl;
        //     return -1;
        // }

        Mat binary = preprocessImage(frame);
        vector<RotatedRect> light_bars = findLightBars(binary);

        imshow("binary", binary);   
        
        vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);
        Mat armor_pair_vis = frame.clone();

        if (armor_pairs.empty()) {
            // 没有检测到装甲板，使用跟踪器预测
            if (!is_first_frame) {
                cv::Point3f predicted_pos = armor_tracker.getPredictedPosition();
                cv::Mat prediction = rotation_center_kf.predict();
                cv::Point3f predicted_rc(prediction.at<float>(0), 
                                        prediction.at<float>(1), 
                                        prediction.at<float>(2));
                
                cout << "Frame " << frame_count << ": Tracking Predicted - " 
                     << "Pos: " << predicted_pos 
                     << " RC: " << predicted_rc << endl;
            }
            continue;
        }

        vector<Point2f> armor_corners = getArmorCorners(armor_pairs[0]);
        Mat rvec, tvec;
        solveArmorPose(armor_corners, rvec, tvec);
        cv::Point3f current_position(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        // 更新跟踪器
        cv::Point3f predicted_position = armor_tracker.update(current_position, timestamp);

        // 卡尔曼滤波处理旋转中心
        if (is_first_frame) {
            // 第一帧初始化
            cv::Mat initial_state = (cv::Mat_<float>(6,1) << 
                current_rotation_center.x, current_rotation_center.y, current_rotation_center.z,
                0.0f, 0.0f, 0.0f);
            rotation_center_kf.init(initial_state);
            is_first_frame = false;
        } else {
            // 预测步骤
            cv::Mat prediction = rotation_center_kf.predict();
            cv::Point3f predicted_rc(prediction.at<float>(0), 
                                    prediction.at<float>(1), 
                                    prediction.at<float>(2));
            
            // 校正步骤
            cv::Mat measurement = (cv::Mat_<float>(3,1) << 
                current_rotation_center.x, 
                current_rotation_center.y, 
                current_rotation_center.z);
            rotation_center_kf.correct(measurement);
        }

        // 获取卡尔曼滤波状态（使用新增的get方法）
        cv::Mat filtered_state = rotation_center_kf.getStatePost();
        cv::Point3f filtered_rc(filtered_state.at<float>(0),
                       filtered_state.at<float>(1),
                       filtered_state.at<float>(2));
        // 输出结果
        cout << "Frame " << frame_count << ":\n"
             << "  Position: " << current_position << " (Pred: " << predicted_position << ")\n"
             << "  Rotation Center: " << current_rotation_center 
             << " (Filtered: " << filtered_rc << ")\n";

        // 坐标系转换与输出
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // 旋转矩阵转欧拉角（yaw, pitch, roll）
        double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0));
        bool singular = sy < 1e-6;
        double roll, pitch, yaw;
        if (!singular) {
            roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
            pitch = atan2(-R.at<double>(2,0), sy);
            yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
        } else {
            roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            pitch = atan2(-R.at<double>(2,0), sy);
            yaw = 0;
        }
        double tx = tvec.at<double>(0);
        double ty = tvec.at<double>(1);
        double tz = tvec.at<double>(2);

        Pose camera_pose(tx, ty, tz, yaw, pitch, roll); // x, y, z, yaw, pitch, roll
        CoordinateTransformer transformer;
        Pose gimbal_pose = transformer.transformToTarget(camera_pose, "/Gimbal");
        auto pos = gimbal_pose.getPosition();
        auto orient = gimbal_pose.getOrientation();
        std::cout << "第 " << frame_count << " 帧 Gimbal系下坐标: "
                  << "x=" << pos[0] << " y=" << pos[1] << " z=" << pos[2]
                  << " yaw=" << orient[0] << " pitch=" << orient[1] << " roll=" << orient[2]
                  << std::endl;

        Point2f center = (armor_corners[0] + armor_corners[2]) * 0.5f;
        drawDistanceInfo(frame, norm(tvec), armor_corners);

        // 计算角速度和线速度
        MotionEstimator::MotionState state = motion_estimator.update(rvec, tvec);
        if(state.linear_velocity.x != 0 || state.linear_velocity.y != 0 || state.linear_velocity.z != 0){
        std::cout << "线速度: " << state.linear_velocity << std::endl;
        std::cout << "角速度: " << state.angular_velocity << std::endl;
        }
        // 计算旋转中心
        RotationCenterCalculator rotation_center_calculator;
        cv::Point3f rotation_center = rotation_center_calculator.calculateRotationCenter(armor_corners, camera_matrix, dist_coeffs, rotation_radius);
        std::cout << "Rotation Center: (" << rotation_center.x << ", " << rotation_center.y << ", " << rotation_center.z << ")" << std::endl;


       // 可视化
        drawDistanceInfo(frame, norm(tvec), armor_corners);
        drawRotationCenter(frame, filtered_rc, camera_matrix, dist_coeffs);
        imshow("Result", frame);
        
        last_timestamp = timestamp;
        if (waitKey(30) == 27) break;
    }
    
    cap.release();
    destroyAllWindows();
    return 0;
}