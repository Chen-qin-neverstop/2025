#pragma once

#include "Protocol.h"
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <queue>
#include <functional>

struct TransformData {
    double x, y, z;
    double roll, pitch, yaw;
    uint32_t data_id;
};

class VideoClient {
public:
    VideoClient(const std::string& server_ip, int server_port, int worker_threads);
    ~VideoClient();

    void start();
    void stop();
    bool isRunning() const;
    void run();
    TransformData getLatestTransform(uint32_t timeout_ms = 1000);

private:
    void handleTransformMessage(const MessageBuffer& msg);
    void handleStringMessage(const MessageBuffer& msg);
    void videoCaptureThread();
    void networkThread();
    void processingThread();
    void cleanup();

    // 线程池实现
    class ThreadPool {
    public:
        explicit ThreadPool(int num_threads) : stop_(false) {
            for (int i = 0; i < num_threads; ++i) {
                workers_.emplace_back([this] {
                    while (true) {
                        std::function<void()> task;
                        {
                            std::unique_lock<std::mutex> lock(queue_mutex_);
                            condition_.wait(lock, [this] {
                                return stop_ || !tasks_.empty();
                            });
                            if (stop_ && tasks_.empty()) {
                                return;
                            }
                            task = std::move(tasks_.front());
                            tasks_.pop();
                        }
                        task();
                    }
                });
            }
        }

        ~ThreadPool() {
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                stop_ = true;
            }
            condition_.notify_all();
            for (std::thread& worker : workers_) {
                if (worker.joinable()) {
                    worker.join();
                }
            }
        }

        template<class F>
        void enqueue(F&& f) {
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                if (stop_) {
                    throw std::runtime_error("enqueue on stopped ThreadPool");
                }
                tasks_.emplace(std::forward<F>(f));
            }
            condition_.notify_one();
        }

    private:
        std::vector<std::thread> workers_;
        std::queue<std::function<void()>> tasks_;
        std::mutex queue_mutex_;
        std::condition_variable condition_;
        std::atomic<bool> stop_;
    };

    std::string server_ip_;
    int server_port_;
    int client_socket_ = -1;
    std::atomic<bool> running_ = false;
    std::atomic<bool> threads_initialized_ = false;

    // 线程相关
    std::thread network_thread_;
    std::thread video_thread_;
    std::vector<std::thread> worker_threads_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable transform_cv_;

    // 线程池
    std::unique_ptr<ThreadPool> thread_pool_;

    // 数据
    TransformData latest_transform_{0, 0, 0, 0, 0, 0, 0};
}; 