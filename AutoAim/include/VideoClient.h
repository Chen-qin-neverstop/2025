#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <opencv2/opencv.hpp>

namespace protocol {
    struct TransformData {
        double x, y, z, roll, pitch, yaw;
        uint32_t data_id;
    };
}

class VideoClient {
public:
    VideoClient();
    ~VideoClient();

    void start();
    void stop();
    void run();

private:
    void videoCaptureThread(); // 图像处理线程，负责捕获视频帧，并调用图像处理（二值化、轮廓检测、最小外接矩形）后，将装甲板角点（即旋转中心）存入 armor_corners_，并通知预测线程。
    void transformThread();     // 位置转换线程，负责将装甲板角点（即旋转中心）转换为世界坐标（例如，利用仿射变换或PnP），并输出文本格式。
    void predictThread();       // 旋转中心预测线程，负责根据 armor_corners_ 预测装甲板旋转中心，并输出文本格式。
    void cleanup();

    std::atomic<bool> running_{false};
    std::atomic<bool> cleaned_up_{false};
    std::atomic<bool> threads_initialized_{false};
    std::thread::id main_thread_id_;
    std::thread video_thread_; // 图像处理线程
    std::thread transform_thread_; // 位置转换线程
    std::thread predict_thread_; // 旋转中心预测线程
    std::condition_variable cv_; // 用于通知预测线程有新的装甲板角点
    std::mutex mutex_; // 互斥锁，保护 armor_corners_ 和 has_new_corners_
    std::vector<cv::Point2f> armor_corners_; // 存储当前装甲板角点（即旋转中心）
    std::atomic<bool> has_new_corners_{false}; // 标记是否有新的装甲板角点数据
}; 