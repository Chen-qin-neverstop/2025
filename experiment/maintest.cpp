#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"

using namespace std;
using namespace cv;

// 全局阈值变量
int thres_max_color_blue = 38;
int gray_threshold = 136;

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

struct TestResult {
    int threshold;
    int detection_count;
    float detection_rate;
};

void runThresholdTest(const string& video_path, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    ofstream result_file("threshold_test_results.csv");
    result_file << "Threshold,Detection Count,Detection Rate(%)\n";
    
    vector<TestResult> results;
    int max_detection = 0;
    float max_rate = 0;
    int best_threshold = 0;
    
    // 灰度阈值范围100-150
    for (int test_threshold = 100; test_threshold <= 150; test_threshold++) {
        gray_threshold = test_threshold;
        
        VideoCapture cap(video_path);
        if (!cap.isOpened()) {
            cerr << "无法打开视频: " << video_path << endl;
            continue;
        }

        MotionEstimator motion_estimator;
        RotationCenterCalculator rotation_center_calculator;
        int armor_detection_count = 0;
        int frame_count = 0;
        Mat frame;
        
        while (true) {
            cap >> frame;
            if (frame.empty()) break;
            
            frame_count++;
            
            Mat binary = preprocessImage(frame);
            vector<RotatedRect> light_bars = findLightBars(binary);
            vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);
            
            if (!armor_pairs.empty()) {
                armor_detection_count++;
                
                vector<Point2f> armor_corners = getArmorCorners(armor_pairs[0]);
                Mat rvec, tvec;
                solveArmorPose(armor_corners, rvec, tvec);
                
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                
                double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
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
                
                MotionEstimator::MotionState state = motion_estimator.update(rvec, tvec);
                cv::Point3f rotation_center = RotationCenterCalculator::Calculate(gimbal_pose, state.linear_velocity, state.angular_velocity);
                
                drawRotationCenter(frame, rotation_center, camera_matrix, dist_coeffs, 0);
                drawDistanceInfo(frame, norm(tvec), armor_corners);
            }
        }
        
        cap.release();
        
        float detection_rate = (frame_count > 0) ? (static_cast<float>(armor_detection_count) / frame_count * 100) : 0;
        
        TestResult result;
        result.threshold = test_threshold;
        result.detection_count = armor_detection_count;
        result.detection_rate = detection_rate;
        results.push_back(result);
        
        result_file << test_threshold << "," << armor_detection_count << "," << fixed << setprecision(2) << detection_rate << "\n";
        
        if (detection_rate > max_rate) {
            max_rate = detection_rate;
            max_detection = armor_detection_count;
            best_threshold = test_threshold;
        }
    }
    
    result_file.close();
    
    cout << "\n=============== 测试结果汇总 ===============" << endl;
    cout << "测试阈值范围: 100-150" << endl;
    cout << "最佳阈值: " << best_threshold << endl;
    cout << "最大检测次数: " << max_detection << endl;
    cout << "最高检测率: " << fixed << setprecision(2) << max_rate << "%" << endl;
    cout << "结果已保存到 threshold_test_results.csv" << endl;
    cout << "=========================================" << endl;
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

    string video_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/2.mp4";
    
    // 运行阈值测试
    runThresholdTest(video_path, camera_matrix, dist_coeffs);
    
    return 0;
}