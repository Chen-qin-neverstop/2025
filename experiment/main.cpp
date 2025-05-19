#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
#include "KalmanFilter.h"    
#include "ArmorTracker.h"    // 还在测试中，暂时不使用
#include "DataBuffer.h"
#include <atomic>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include "Visualizer.h"

using namespace std;
using namespace cv;


// 全局共享数据
ThreadSafeQueue<FrameData> frame_data_queue;  // 线程安全帧数据队列
DoubleBuffer frame_double_buffer;             // 双缓冲用于处理结果
atomic<bool> stop_flag(false);               // 线程停止标志


double timestamp = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();

void processing_thread() {
    try {
        while (!stop_flag) {
            FrameData frame_data;
            if (frame_data_queue.try_pop(frame_data)) {
                // 1. 图像预处理
                Mat binary = preprocessImage(frame_data.frame);
                
                // 2. 灯条检测
                vector<RotatedRect> light_bars = findLightBars(binary);
                
                // 3. 将处理结果存入双缓冲
                if (!frame_double_buffer.try_push(std::move(binary))) {
                    cerr << "Double buffer full, dropping frame" << endl;
                }
            }
            this_thread::sleep_for(1ms); // 避免CPU占用过高
        }
    } catch (const exception& e) {
        cerr << "Processing thread error: " << e.what() << endl;
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
    RotationCenterKalmanFilter rc_kalman; // 旋转中心专用卡尔曼滤波器
    cv::Point3f last_valid_rc; // 记录最后有效旋转中心
    const float MAX_JUMP_DISTANCE = 0.4f; // 最大允许跳变距离(米)
    MotionEstimator motion_estimator; // 在while循环之前声明
    RotationCenterCalculator rotation_center_calculator;
    cv::Point3f current_rotation_center;
    

    // 改为读取视频
    std::string video_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/1.mp4"; 
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "无法打开视频: " << video_path << endl;
        return -1;
    }

    // 启动处理线程
    std::thread processor(processing_thread);

    // 主循环
    int frame_count = 0;
    Mat frame;

    
    while (!stop_flag) {
        cap >> frame;
        if (frame.empty()) {
            cout << "视频处理完成" << endl;
            break;
        }
        
        frame_count++;
        
        // 2. 将帧数据推入处理队列
        FrameData current_frame;
        current_frame.frame = frame.clone();
        current_frame.frame_count = frame_count;

        frame_data_queue.push(std::move(current_frame));  // 使用std::move

        // 3. 尝试从双缓冲获取处理结果
        if (frame_double_buffer.wait_for_data(10)) {
            auto processed_frames = frame_double_buffer.get_front_buffer();
            if (!processed_frames.empty()) {
                imshow("binary", processed_frames[0]); // 显示处理后的二值图像
            }
            frame_double_buffer.swap_buffers();
        }

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
        
        // 根据世界坐标系下装甲板中心点算出线速度和角速度来推出旋转半径
        // 计算线速度和角速度
        // 9. 运动状态估计
        MotionEstimator::MotionState state = motion_estimator.update(rvec, tvec);
        //计算旋转中心(世界坐标系下)
        cv::Point3f rotation_center = RotationCenterCalculator::Calculate(gimbal_pose, state.linear_velocity, state.angular_velocity);
        // cout << "旋转中心: " << rotation_center << endl;
        
        //10 卡尔曼滤波更新
        try {
            cv::Point3f filtered_rc = rc_kalman.update(rotation_center);   // 这里应该是rotation_center
            
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
            
            // 可视化(用于测试/可删除)
            drawRotationCenter(frame, filtered_rc, camera_matrix, dist_coeffs,1);  
            RotationCenterData current_data;
            current_data.frame = frame_count;
            current_data.raw_center = rotation_center;
            current_data.filtered_center = filtered_rc;
            current_data.timestamp = timestamp;

            // visualizer.addDataPoint(current_data);
            // visualizer.showCurrentFrame(frame, current_data); 
            
        } catch (const std::exception& e) {
            cerr << "Kalman update error: " << e.what() << endl;
            rc_kalman.init(rotation_center); 
            last_valid_rc = rotation_center;  
        }

       // 可视化（先别删）
        drawRotationCenter(frame, rotation_center, camera_matrix, dist_coeffs,0);
        drawDistanceInfo(frame, norm(tvec), armor_corners);
        drawArmorCenter(frame, current_rotation_position, camera_matrix, dist_coeffs,0);
        RotationCenterData current_data;

        imshow("Result", frame);
        
        //last_timestamp = timestamp;
        if (waitKey(30) == 27)
        {
            stop_flag = true;
            break;
        }
    }
    
    // ================= 资源清理 =================
    frame_data_queue.stop();      // 停止队列
    frame_double_buffer.stop();   // 停止双缓冲
    processor.join();            // 等待处理线程结束
    cap.release();               // 释放视频资源
    destroyAllWindows();         // 关闭所有窗口
    cout << "程序退出" << endl;
    return 0;
}