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

// 全局阈值变量声明（在ImageProcess.cpp中定义）
extern int thres_max_color_blue;
extern int gray_threshold;

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
    int blue_threshold;
    int gray_threshold;
    int detection_count;
    float detection_rate;
};

void runThresholdTest(const string& video_path, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    ofstream result_file("threshold_test_results.csv");
    result_file << "Blue Threshold,Gray Threshold,Detection Count,Detection Rate(%)\n";
    
    vector<TestResult> results;
    int max_detection = 0;
    float max_rate = 0;
    int best_blue_threshold = 0;
    int best_gray_threshold = 0;
    
    // 蓝色阈值范围 (示例范围，可根据需要调整)
    for (int blue_thresh = 30; blue_thresh <= 50; blue_thresh += 1) {
        // 灰度阈值范围100-150
        for (int gray_thresh = 120; gray_thresh <= 140; gray_thresh += 2) {
            thres_max_color_blue = blue_thresh;
            gray_threshold = gray_thresh;
            
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
            }
            }
            
            cap.release();
            
            float detection_rate = (frame_count > 0) ? (static_cast<float>(armor_detection_count) / frame_count * 100) : 0;
            
            TestResult result;
            result.blue_threshold = blue_thresh;
            result.gray_threshold = gray_thresh;
            result.detection_count = armor_detection_count;
            result.detection_rate = detection_rate;
            results.push_back(result);
             

            result_file << blue_thresh << "," << gray_thresh << "," 
                       << armor_detection_count << "," << fixed << setprecision(2) << detection_rate << "\n";
            
            if (detection_rate > max_rate) {
                max_rate = detection_rate;
                max_detection = armor_detection_count;
                best_blue_threshold = blue_thresh;
                best_gray_threshold = gray_thresh;
            }
        }
    }
    
    result_file.close();
    
    cout << "\n=============== 测试结果汇总 ===============" << endl;
    cout << "蓝色阈值测试范围: 10-60" << endl;
    cout << "灰度阈值测试范围: 100-150" << endl;
    cout << "最佳蓝色阈值: " << best_blue_threshold << endl;
    cout << "最佳灰度阈值: " << best_gray_threshold << endl;
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