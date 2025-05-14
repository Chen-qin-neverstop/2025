#include "VideoClient.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <functional>
#include <atomic>

// ThreadPool 实现
class ThreadPool {
public:
    ThreadPool(size_t numThreads) : stop(false) {
        for (size_t i = 0; i < numThreads; ++i) {
            threads.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queueMutex);
                        this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty()) {
                            return;
                        }
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& thread : threads) {
            thread.join();
        }
    }

    template<class F, class... Args>
    void enqueue(F&& f, Args&&... args) {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            tasks.emplace(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
        }
        condition.notify_one();
    }

private:
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::atomic<bool> stop;
};

// VideoClient 实现
VideoClient::VideoClient(const std::string& server_ip, int server_port, int worker_threads)
    : server_ip_(server_ip), server_port_(server_port) {
    thread_pool_ = std::make_unique<ThreadPool>(worker_threads);
}

VideoClient::~VideoClient() {
    stop();
    cleanup();
}

void VideoClient::start() {
    if (!running_) {
        running_ = true;
        network_thread_ = std::thread(&VideoClient::networkThread, this);
        for (size_t i = 0; i < worker_threads_.size(); ++i) {
            worker_threads_[i] = std::thread(&VideoClient::processingThread, this);
        }
        threads_initialized_ = true;
    }
}

void VideoClient::stop() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (threads_initialized_) {
            if (network_thread_.joinable()) network_thread_.join();
            for (auto& thread : worker_threads_) {
                if (thread.joinable()) thread.join();
            }
        }
    }
}

bool VideoClient::isRunning() const {
    return running_;
}

void VideoClient::run() {
    start();
    
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        throw std::runtime_error("Socket creation failed");
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port_);
    
    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
        throw std::runtime_error("Invalid address");
    }

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        throw std::runtime_error("Connection failed");
    }

    client_socket_ = sock;
    std::cout << "Connected to server at " << server_ip_ << ":" << server_port_ << std::endl;

    // 启动视频捕获线程
    video_thread_ = std::thread(&VideoClient::videoCaptureThread, this);

    // 主循环
    while (running_) {
        try {
            MessageBuffer msg;
            ssize_t bytes_read = recv(client_socket_, &msg, sizeof(MessageBuffer), 0);
            
            if (bytes_read <= 0) {
                if (running_) {
                    std::cerr << "Connection closed by server" << std::endl;
                }
                break;
            }

            if (!Protocol::validateMessage(msg)) {
                std::cerr << "Invalid message received" << std::endl;
                continue;
            }

            switch (msg.MessageType) {
                case TRANSFORM: {
                    handleTransformMessage(msg);
                    break;
                }
                case STRING_MSG: {
                    handleStringMessage(msg);
                    break;
                }
                default: {
                    std::cerr << "Unsupported message type: " << msg.MessageType << std::endl;
                    break;
                }
            }
        } catch (const std::exception& e) {
            if (running_) {
                std::cerr << "Error in main loop: " << e.what() << std::endl;
            }
            break;
        }
    }
}

void VideoClient::handleTransformMessage(const MessageBuffer& msg) {
    double x, y, z, roll, pitch, yaw;
    Protocol::extractTransformData(msg, x, y, z, roll, pitch, yaw);
    
    std::lock_guard<std::mutex> lock(mutex_);
    latest_transform_ = TransformData{x, y, z, roll, pitch, yaw, msg.DataID};
    transform_cv_.notify_one();
}

void VideoClient::handleStringMessage(const MessageBuffer& msg) {
    // 使用 Data 成员和 DataLength 成员来提取字符串数据
    std::string str_msg(reinterpret_cast<const char*>(msg.Data), msg.DataLength);
    std::cout << "Received string message: " << str_msg << std::endl;
}

void VideoClient::videoCaptureThread() {
    std::cout << "Attempting to open video source..." << std::endl;
    
    cv::VideoCapture cap;
    // 尝试打开视频文件，如果失败则尝试打开摄像头
    if (!cap.open("/home/chen/视频/video/videos/2.mp4")) {  // 首先尝试打开视频文件
        std::cout << "Failed to open video file, trying camera..." << std::endl;
        // 尝试不同的摄像头设备号
        for (int device = 0; device < 2; device++) {
            std::cout << "Trying camera device " << device << std::endl;
            if (cap.open(device)) {
                std::cout << "Successfully opened camera device " << device << std::endl;
                break;
            }
        }
    } else {
        std::cout << "Successfully opened video file" << std::endl;
    }

    if (!cap.isOpened()) {
        std::cerr << "Failed to open any video source. Please check:" << std::endl;
        std::cerr << "1. Video file exists and is valid" << std::endl;
        std::cerr << "2. Camera is properly connected" << std::endl;
        std::cerr << "3. No other program is using the camera" << std::endl;
        std::cerr << "4. You have proper permissions to access the camera" << std::endl;
        throw std::runtime_error("Failed to open video source");
    }

    // 获取并打印视频信息
    double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = cap.get(cv::CAP_PROP_FPS);
    double total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout << "Video properties:" << std::endl;
    std::cout << "Resolution: " << actual_width << "x" << actual_height << std::endl;
    std::cout << "FPS: " << actual_fps << std::endl;
    if (total_frames > 0) {
        std::cout << "Total frames: " << total_frames << std::endl;
    }

    // 创建显示窗口
    cv::namedWindow("Video Client", cv::WINDOW_NORMAL);
    cv::resizeWindow("Video Client", actual_width, actual_height);

    uint32_t frame_id = 0;
    int failed_frames = 0;
    double last_frame_time = cv::getTickCount() / cv::getTickFrequency();
    
    while (running_) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            if (total_frames > 0) {  // 如果是视频文件，循环播放
                std::cout << "End of video, restarting..." << std::endl;
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            } else {
                std::cerr << "Failed to capture frame " << ++failed_frames << std::endl;
                if (failed_frames > 10) {
                    std::cerr << "Too many failed frames, stopping capture" << std::endl;
                    break;
                }
                continue;
            }
        }
        failed_frames = 0;  // 重置失败计数

        if (frame.empty()) {
            std::cerr << "Received empty frame" << std::endl;
            continue;
        }

        // 计算实际帧率
        double current_time = cv::getTickCount() / cv::getTickFrequency();
        double fps = 1.0 / (current_time - last_frame_time);
        last_frame_time = current_time;

        // 在图像上显示帧率
        std::string fps_text = "FPS: " + std::to_string(int(fps));
        cv::putText(frame, fps_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // 显示视频帧
        cv::imshow("Video Client", frame);
        char key = cv::waitKey(1);  // 等待1ms，处理窗口事件
        if (key == 27) {  // ESC键退出
            std::cout << "ESC key pressed, stopping capture" << std::endl;
            running_ = false;
            break;
        }

        // 将图像分块发送到服务器
        std::vector<MessageBuffer> messages = Protocol::createImageMessage(frame, frame_id++);
        for (const auto& msg : messages) {
            if (send(client_socket_, &msg, sizeof(MessageBuffer), 0) < 0) {
                if (running_) {
                    std::cerr << "Failed to send image data" << std::endl;
                }
                break;
            }
        }

        // 如果是视频文件，按照视频的原始帧率播放
        if (total_frames > 0 && actual_fps > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0/actual_fps)));
        } else {
            // 如果是摄像头，控制发送速率（约30FPS）
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }

    std::cout << "Cleaning up video capture..." << std::endl;
    cap.release();
    cv::destroyWindow("Video Client");
    std::cout << "Video capture cleanup completed" << std::endl;
}

TransformData VideoClient::getLatestTransform(uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (transform_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this] { return !running_ || latest_transform_.data_id != 0; })) {
        return latest_transform_;
    }
    return TransformData{0, 0, 0, 0, 0, 0, 0};  // 超时返回空数据
}

void VideoClient::cleanup() {
    if (client_socket_ >= 0) {
        close(client_socket_);
        client_socket_ = -1;
    }
}

void VideoClient::networkThread() {
    while (running_) {
        try {
            MessageBuffer msg;
            ssize_t bytes_read = recv(client_socket_, &msg, sizeof(MessageBuffer), 0);
            
            if (bytes_read <= 0) {
                if (running_) {
                    std::cerr << "Connection closed by server" << std::endl;
                }
                break;
            }

            if (!Protocol::validateMessage(msg)) {
                std::cerr << "Invalid message received" << std::endl;
                continue;
            }

            // 将消息加入处理队列
            thread_pool_->enqueue([this, msg]() {
                switch (msg.MessageType) {
                    case TRANSFORM: {
                        handleTransformMessage(msg);
                        break;
                    }
                    case STRING_MSG: {
                        handleStringMessage(msg);
                        break;
                    }
                    default: {
                        std::cerr << "Unsupported message type: " << msg.MessageType << std::endl;
                        break;
                    }
                }
            });
        } catch (const std::exception& e) {
            if (running_) {
                std::cerr << "Error in network thread: " << e.what() << std::endl;
            }
            break;
        }
    }
}

void VideoClient::processingThread() {
    while (running_) {
        try {
            // 处理线程可以在这里执行其他任务
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } catch (const std::exception& e) {
            if (running_) {
                std::cerr << "Error in processing thread: " << e.what() << std::endl;
            }
            break;
        }
    }
}

int main(int argc, char** argv) {
    try {
        std::string server_ip = "127.0.0.1";
        int server_port = 8080;
        size_t numThreads = 4;

        if (argc > 1) {
            server_ip = argv[1];
        }
        if (argc > 2) {
            server_port = std::stoi(argv[2]);
        }
        if (argc > 3) {
            numThreads = std::stoul(argv[3]);
        }

        VideoClient client(server_ip, server_port, numThreads);
        client.run();
    } catch (const std::exception& e) {
        std::cerr << "Client error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 