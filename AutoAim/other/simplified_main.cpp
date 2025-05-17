#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"

using namespace std;
using namespace cv;

void drawRotationCenter(cv::Mat& frame, const cv::Point3f& center, 
                       const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, int color) {
    std::vector<cv::Point3f> points{center};
    std::vector<cv::Point2f> projected_points;
    
    cv::projectPoints(points, cv::Mat::zeros(3,1,CV_32F), cv::Mat::zeros(3,1,CV_32F),
                     camera_matrix, dist_coeffs, projected_points);
    
    if (!projected_points.empty()) {
        if(color == 0){    // 绿色
            cv::circle(frame, projected_points[0], 10, cv::Scalar(0, 255, 0), 2);
        }
        else if(color == 1){   // 红色
            cv::circle(frame, projected_points[0], 10, cv::Scalar(0, 0, 255), 2);
        }
        cv::putText(frame, "RC: " + std::to_string(center.x) + "," + std::to_string(center.y), 
                   projected_points[0] + cv::Point2f(15,0), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
    }
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

    // 初始化组件
    MotionEstimator motion_estimator;
    RotationCenterCalculator rotation_center_calculator;
    
    // 读取视频
    std::string video_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/2.mp4"; 
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "无法打开视频: " << video_path << endl;
        return -1;
    }

    Mat frame;
    int frame_count = 0;
    int armor_count = 0;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cout << "视频处理完成" << endl;
            break;
        }
        
        frame_count++;
        
        // 1. 图像预处理
        Mat binary = preprocessImage(frame);
        
        // 2. 灯条检测
        vector<RotatedRect> light_bars = findLightBars(binary);
        
        // 3. 装甲板匹配
        vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);
        
        if (armor_pairs.empty()) {
            imshow("Result", frame);
            if (waitKey(30) == 27) break;
            continue;
        }
        
        // 4. 获取装甲板角点并解算位姿
        vector<Point2f> armor_corners = getArmorCorners(armor_pairs[0]);
        Mat rvec, tvec;
        solveArmorPose(armor_corners, rvec, tvec);
        
        // 5. 坐标系转换
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
        
        Pose camera_pose(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), yaw, pitch, roll);
        CoordinateTransformer transformer;
        Pose gimbal_pose = transformer.transformToTarget(camera_pose, "/Gimbal");
        
        // 6. 运动状态估计
        MotionEstimator::MotionState state = motion_estimator.update(rvec, tvec);
        
        // 7. 计算旋转中心
        cv::Point3f rotation_center = RotationCenterCalculator::Calculate(gimbal_pose, state.linear_velocity, state.angular_velocity);
        
        // 8. 可视化
        drawRotationCenter(frame, rotation_center, camera_matrix, dist_coeffs, 0);
        drawDistanceInfo(frame, norm(tvec), armor_corners);
        imshow("Result", frame);
        imshow("binary", binary);
        
        if (waitKey(30) == 27) break;
    }
    
    cap.release();
    destroyAllWindows();
    cout << "程序退出" << endl;
    return 0;
}