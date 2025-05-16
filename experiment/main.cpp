#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
using namespace std;
using namespace cv;

int main(int argc, char** argv) {
// 相机参数
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
    1000.0, 0.0, 640.0,
    0.0, 1000.0, 360.0,
    0.0, 0.0, 1.0);
// 畸变系数
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 
    0.0, 0.0, 0.0, 0.0, 0.0);
// 旋转半径需要根据目标实际尺寸设置（示例值，单位：米）
float rotation_radius = 0.5;

    // 改为读取视频
    std::string video_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/2.mp4"; // 修改为你的视频路径
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "无法打开视频: " << video_path << endl;
        return -1;
    }

    int frame_count = 0;
    Mat frame;
    
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
            // cout << "第 " << frame_count << " 帧未检测到装甲板" << endl;
            continue;
        }

        vector<Point2f> armor_corners = getArmorCorners(armor_pairs[0]);
        Mat rvec, tvec;
        solveArmorPose(armor_corners, rvec, tvec);

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
        MotionEstimator motion_estimator;
        MotionEstimator::MotionState state = motion_estimator.update(rvec, tvec);
        // std::cout << "线速度: " << state.linear_velocity << std::endl;
        // std::cout << "角速度: " << state.angular_velocity << std::endl;

        // 计算旋转中心
        RotationCenterCalculator rotation_center_calculator;
        cv::Point3f rotation_center = rotation_center_calculator.calculateRotationCenter(armor_corners, camera_matrix, dist_coeffs, rotation_radius);
        std::cout << "Rotation Center: (" << rotation_center.x << ", " << rotation_center.y << ", " << rotation_center.z << ")" << std::endl;

        imshow("Result", frame);
        
        // 按ESC键退出
        if (waitKey(30) == 27) {
            break;
        }
    }
    
    cap.release();
    destroyAllWindows();
    return 0;
}