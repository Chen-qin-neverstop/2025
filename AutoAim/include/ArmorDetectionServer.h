#pragma once

#include <thread>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <queue>
#include <functional>
#include <condition_variable>
#include "ThreadSafeQueue.h"
#include "ArmorTracker.h"
#include "Protocol.h"
#include "ImageProcess.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
#include <string>
#include <opencv2/opencv.hpp>

using namespace protocol;  // 添加命名空间使用

// 前向声明
class ThreadPool;

class ArmorDetectionServer {
public:
    struct Task {
        int dataID;
        cv::Mat image;
        double timestamp;
    };

    struct Result {
        int dataID;
        cv::Point3f position;
        cv::Point3f predicted_position;
        double timestamp;
    };

    ArmorDetectionServer(int port = 8080, int worker_threads = 4);
    ~ArmorDetectionServer();

    void start();
    void stop();
    bool isRunning() const;
    void run();  // 添加 run() 函数声明

private:
    void networkThread();
    void processingThread();
    void predictionThread();
    void cleanup();
    void handleClient(int client_socket);
    void handleImageMessage(int client_socket, const MessageBuffer& firstMsg);
    void processImageAndSendResult(int client_socket, const cv::Mat& image, uint32_t dataID);
    void handleTransformRequest(int client_socket, const MessageBuffer& msg);
    void rotationMatrixToEulerAngles(const cv::Mat &R, double &roll, double &pitch, double &yaw);

    int port_;
    int server_fd_;
    std::atomic<bool> running_{false};
    std::atomic<bool> threads_initialized_{false};

    std::thread network_thread_;
    std::vector<std::thread> worker_threads_;
    std::thread prediction_thread_;

    ThreadSafeQueue<Task> processing_queue_;
    ThreadSafeQueue<Result> prediction_queue_;
    ThreadSafeQueue<Result> output_queue_;

    std::mutex tracker_mutex_;
    std::unordered_map<int, std::unique_ptr<ArmorTracker>> trackers_;

    // 线程同步
    std::condition_variable cv_;
    std::mutex startup_mutex_;

    // 新增成员变量
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    RotationCenterCalculator rotation_center_calculator_;
    std::unordered_map<uint32_t, MotionEstimator> motion_estimators_;
    std::unordered_map<uint32_t, cv::Point3f> rotation_centers_;
    std::mutex mutex_;
    std::unique_ptr<ThreadPool> thread_pool_;  // 使用智能指针管理线程池
};