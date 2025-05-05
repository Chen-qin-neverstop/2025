#ifndef ARMOR_DETECTION_SERVER_H
#define ARMOR_DETECTION_SERVER_H

#include "ThreadSafeQueue.h"
#include "ArmorTracker.h"
#include "Protocol.h"
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>

class ArmorDetectionServer {
public:
    ArmorDetectionServer(int port, int worker_threads = 4);
    ~ArmorDetectionServer();
    
    void start();
    void stop();

private:
    void networkThread();
    void processingThread();
    void predictionThread();
    
    struct Task {
        uint32_t dataID;
        cv::Mat image;
        double timestamp;
    };
    
    struct Result {
        uint32_t dataID;
        cv::Point3f position;
        cv::Point3f predicted_position;
        double timestamp;
    };
    
    // 线程间通信队列
    ThreadSafeQueue<Task> processing_queue_;
    ThreadSafeQueue<Result> prediction_queue_;
    ThreadSafeQueue<Result> output_queue_;
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::vector<std::thread> worker_threads_;
    std::thread network_thread_;
    std::thread prediction_thread_;
    
    // 资源
    int port_;
    int server_fd_;
    std::unordered_map<uint32_t, std::unique_ptr<ArmorTracker>> trackers_;
    std::mutex tracker_mutex_;
};

#endif // ARMOR_DETECTION_SERVER_H