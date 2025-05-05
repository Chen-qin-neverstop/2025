#ifndef ARMOR_DETECTION_SERVER_H
#define ARMOR_DETECTION_SERVER_H

#include <thread>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include "ThreadSafeQueue.h"
#include "ArmorTracker.h"
#include "Protocol.h"
#include "ImageProcess.h"

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

private:
    void networkThread();
    void processingThread();
    void predictionThread();
    void cleanup();

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
};

#endif // ARMOR_DETECTION_SERVER_H