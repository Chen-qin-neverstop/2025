// 已添加kalman滤波，但是没有使用多线程和双缓冲




#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
#include "KalmanFilter.h"    
#include "ArmorTracker.h"    // 还在测试中，暂时不使用
#include "DataBuffer.h"


using namespace std;
using namespace cv;



double timestamp = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();

void drawRotationCenter(cv::Mat& frame, const cv::Point3f& center, 
                       const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,int color) {
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
    //ArmorTracker armor_tracker;
    //MotionEstimator motion_estimator;
    //RotationCenterCalculator rotation_center_calculator;
    // 卡尔曼滤波器
    RotationCenterKalmanFilter rc_kalman; // 旋转中心专用卡尔曼滤波器
    cv::Point3f last_valid_rc; // 记录最后有效旋转中心
    const float MAX_JUMP_DISTANCE = 0.3f; // 最大允许跳变距离(米)

    // 改为读取视频
    std::string video_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/9.mp4"; 
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
        
        vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);  // armor_pairs[0] 储存的是左右灯条  其中每个RotatedRect储存的是灯条的中心点，长宽，倾斜角度等信息
        Mat armor_pair_vis = frame.clone();

        if (armor_pairs.empty()) {
            // 无检测时使用预测值
            try {
                cv::Point3f predicted_rc = rc_kalman.predict();
                drawRotationCenter(frame, predicted_rc, camera_matrix, dist_coeffs,1);
                cout << "Frame " << frame_count << ": Predicted RC - " << predicted_rc << endl;
            } catch (const std::exception& e) {
                cerr << "Prediction error: " << e.what() << endl;
            }
            continue;
        }
        
        vector<Point2f> armor_corners = getArmorCorners(armor_pairs[0]);   //  传入左右灯条的位置用来解算装甲板中心   
        Mat rvec, tvec;
        solveArmorPose(armor_corners, rvec, tvec);
        cv::Point3f current_rotation_position(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));    // 当前装甲板中心点（但是还不是世界坐标系下的）
        // 需要将当前装甲板中心点转换到世界坐标系下
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
        // std::cout << "第 " << frame_count << " 帧 Gimbal系下坐标: "
        //           << "x=" << pos[0] << " y=" << pos[1] << " z=" << pos[2]
        //           << " yaw=" << orient[0] << " pitch=" << orient[1] << " roll=" << orient[2]
        //           << std::endl;
        // 根据世界坐标系下装甲板中心点算出线速度和角速度来推出旋转半径
        // 计算线速度和角速度
        MotionEstimator::MotionState state = motion_estimator.update(rvec, tvec);

        if(state.linear_velocity.x != 0 || state.linear_velocity.y != 0 || state.linear_velocity.z != 0){
        // cout << "当前帧数: " << frame_count << endl;
        // std::cout << "线速度: " << state.linear_velocity << std::endl;
        // std::cout << "角速度: " << state.angular_velocity << std::endl;
        }
        //计算旋转中心(世界坐标系下)
        cv::Point3f rotation_center = RotationCenterCalculator::Calculate(gimbal_pose, state.linear_velocity, state.angular_velocity);
        // cout << "旋转中心: " << rotation_center << endl;
        
        // 卡尔曼滤波更新
        try {
            cv::Point3f filtered_rc = rc_kalman.update(rotation_center);
            
            // 跳变检测
            if (cv::norm(filtered_rc - last_valid_rc) > MAX_JUMP_DISTANCE) {
                filtered_rc = rc_kalman.predict(); // 使用纯预测值
                cout << "Jump detected! Using prediction." << endl;
            }
            
            last_valid_rc = filtered_rc;
            
            // 输出信息
            cout << "Frame " << frame_count << ":\n"
                 << "  Raw RC: " << rotation_center << "\n"
                 << "  Filtered RC: " << filtered_rc << "\n"
                 << "  Velocity: " << state.linear_velocity << endl;
            
            // 可视化
            drawRotationCenter(frame, filtered_rc, camera_matrix, dist_coeffs,1);
            
        } catch (const std::exception& e) {
            cerr << "Kalman update error: " << e.what() << endl;
            rc_kalman.init(rotation_center); // 重新初始化
            last_valid_rc = rotation_center;
        }

       // 可视化
        drawRotationCenter(frame, rotation_center, camera_matrix, dist_coeffs,0);
        drawDistanceInfo(frame, norm(tvec), armor_corners);
        imshow("Result", frame);
        
        //last_timestamp = timestamp;
        if (waitKey(30) == 27) break;
    }
    
    cap.release();
    destroyAllWindows();
    return 0;
}