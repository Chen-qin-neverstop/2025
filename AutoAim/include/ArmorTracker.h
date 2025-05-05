#ifndef ARMOR_DETECTION_SERVER_H
#define ARMOR_DETECTION_SERVER_H

#include "Protocol.h"
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
#include "ArmorTracker.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

class ArmorDetectionServer {
public:
    ArmorDetectionServer(int port, int worker_threads = 4);
    ~ArmorDetectionServer();
    
    void run();
    void stop();

private:
    struct ClientTask {
        int socket;
        MessageBuffer message;
    };

    void networkThread();
    void processingThread();
    
    void handleClient(int client_socket);
    void handleImageMessage(int client_socket, const MessageBuffer& firstMsg);
    void processImageAndSend(int client_socket, const cv::Mat& image, uint32_t dataID);
    void handleTransformRequest(int client_socket, const MessageBuffer& msg);
    void rotationMatrixToEulerAngles(const cv::Mat &R, double &roll, double &pitch, double &yaw);

    // 线程安全队列
    std::queue<ClientTask> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cond_;
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread network_thread_;
    std::vector<std::thread> worker_threads_;
    
    // 资源
    int port_;
    int server_fd_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    RotationCenterCalculator rotation_center_calculator_;
    
    // 装甲板跟踪器
    std::unordered_map<uint32_t, std::unique_ptr<ArmorTracker>> armor_trackers_;
    std::mutex tracker_mutex_;
    
    // 运动状态记录
    std::unordered_map<uint32_t, MotionEstimator> motion_estimators_;
    std::unordered_map<uint32_t, cv::Point3f> rotation_centers_;
};

#endif // ARMOR_DETECTION_SERVER_H