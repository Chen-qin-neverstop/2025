#include "ArmorDetectionServer.h"
#include "CoordinateTransformer.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <sstream>

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

// ArmorDetectionServer 实现
ArmorDetectionServer::ArmorDetectionServer(int port, int worker_threads) 
    : port_(port), 
      rotation_center_calculator_(0.135f, 0.055f) {
    // 初始化相机参数
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        2065.0580175762857, 0.0, 658.9098266395495,
        0.0, 2086.886458338243, 531.5333174739342,
        0.0, 0.0, 1.0);

    dist_coeffs_ = (cv::Mat_<double>(5, 1) << 
        -0.051836613762195866, 0.29341513924119095, 
        0.001501183796729562, 0.0009386915104617738, 0.0);

    thread_pool_ = std::make_unique<ThreadPool>(worker_threads);
}

ArmorDetectionServer::~ArmorDetectionServer() {
    stop();
    cleanup();
}

void ArmorDetectionServer::start() {
    if (!running_) {
        running_ = true;
        network_thread_ = std::thread(&ArmorDetectionServer::networkThread, this);
        for (size_t i = 0; i < worker_threads_.size(); ++i) {
            worker_threads_[i] = std::thread(&ArmorDetectionServer::processingThread, this);
        }
        prediction_thread_ = std::thread(&ArmorDetectionServer::predictionThread, this);
        threads_initialized_ = true;
    }
}

void ArmorDetectionServer::stop() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (threads_initialized_) {
            if (network_thread_.joinable()) network_thread_.join();
            for (auto& thread : worker_threads_) {
                if (thread.joinable()) thread.join();
            }
            if (prediction_thread_.joinable()) prediction_thread_.join();
        }
    }
}

bool ArmorDetectionServer::isRunning() const {
    return running_;
}

void ArmorDetectionServer::run() {
    start();
    
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        throw std::runtime_error("Socket creation failed");
    }

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        throw std::runtime_error("Bind failed");
    }

    if (listen(server_fd, 3) < 0) {
        throw std::runtime_error("Listen failed");
    }

    server_fd_ = server_fd;
    std::cout << "Server listening on port " << port_ << std::endl;

    while (running_) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_socket < 0) {
            if (running_) {
                std::cerr << "Accept failed" << std::endl;
            }
            continue;
        }

        thread_pool_->enqueue([this, client_socket]() {
            try {
                handleClient(client_socket);
            } catch (const std::exception& e) {
                std::cerr << "Error handling client: " << e.what() << std::endl;
            }
            close(client_socket);
        });
    }
}

void ArmorDetectionServer::handleClient(int client_socket) {
    try {
        MessageBuffer msg;
        ssize_t bytes_read = recv(client_socket, &msg, sizeof(MessageBuffer), 0);
        
        if (bytes_read != sizeof(MessageBuffer)) {
            throw std::runtime_error("Invalid message size received");
        }

        if (!Protocol::validateMessage(msg)) {
            throw std::runtime_error("Invalid message format");
        }

        switch (msg.MessageType) {
            case IMAGE_MSG: {
                handleImageMessage(client_socket, msg);
                break;
            }
            case TRANSFORM_REQUEST: {
                handleTransformRequest(client_socket, msg);
                break;
            }
            default: {
                MessageBuffer response = Protocol::createStringMessage(
                    "Unsupported message type", 0);
                send(client_socket, &response, sizeof(MessageBuffer), 0);
                break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error handling client: " << e.what() << std::endl;
        MessageBuffer errorMsg = Protocol::createStringMessage(
            std::string("Error: ") + e.what(), 0);
        send(client_socket, &errorMsg, sizeof(MessageBuffer), 0);
    }
}

void ArmorDetectionServer::handleImageMessage(int client_socket, const MessageBuffer& firstMsg) {
    std::vector<MessageBuffer> messages;
    messages.push_back(firstMsg);

    while (messages.back().Offset + messages.back().DataLength < messages.back().DataTotalLength) {
        MessageBuffer nextMsg;
        ssize_t bytes_read = recv(client_socket, &nextMsg, sizeof(MessageBuffer), 0);
        
        if (bytes_read != sizeof(MessageBuffer)) {
            throw std::runtime_error("Failed to receive next image chunk");
        }
        
        messages.push_back(nextMsg);
    }

    cv::Mat image = Protocol::extractImage(messages);
    processImageAndSendResult(client_socket, image, firstMsg.DataID);
}

void ArmorDetectionServer::processImageAndSendResult(int client_socket, const cv::Mat& image, uint32_t dataID) {
    try {
        cv::Mat undistorted;
        cv::undistort(image, undistorted, camera_matrix_, dist_coeffs_);
        cv::Mat binary = preprocessImage(undistorted);
        
        auto light_bars = findLightBars(binary);
        auto armor_pairs = matchArmorPairs(light_bars);
        
        for (const auto& pair : armor_pairs) {
            auto corners = getArmorCorners(pair);
            
            cv::Mat rvec, tvec;
            solveArmorPose(corners, rvec, tvec);
            
            cv::Mat rot_mat;
            cv::Rodrigues(rvec, rot_mat);
            
            double roll, pitch, yaw;
            rotationMatrixToEulerAngles(rot_mat, roll, pitch, yaw);
            
            MessageBuffer transformMsg = Protocol::createTransformMessage(
                tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                roll, pitch, yaw, dataID);
            send(client_socket, &transformMsg, sizeof(MessageBuffer), 0);
            // 更新姿态估计器
            std::lock_guard<std::mutex> lock(mutex_);
            auto& estimator = motion_estimators_[dataID];
            auto motion_state = estimator.update(rvec, tvec);
            // 计算转向中心
            if (motion_state.rotation_radius > 0) {
                cv::Point3f rotation_center = rotation_center_calculator_.calculateRotationCenter(
                    corners, camera_matrix_, dist_coeffs_, motion_state.rotation_radius);
                // 存储旋转中心
                rotation_centers_[dataID] = rotation_center;
                
                std::stringstream motionInfo;
                motionInfo << "Motion State - "
                          << "Linear Vel: (" << motion_state.linear_velocity.x << ", "
                          << "Linear Vel: (" << motion_state.linear_velocity.y << ", "
                          << "Linear Vel: (" << motion_state.linear_velocity.z << ") m/s | "
                          << "Angular Vel: (" << motion_state.angular_velocity.x << ", "
                          << "Angular Vel: (" << motion_state.angular_velocity.y << ", "
                          << "Angular Vel: (" << motion_state.angular_velocity.z << ") rad/s | "
                          << "Rotation Radius: " << motion_state.rotation_radius << " m | "
                          << "Rotation Center: (" << rotation_center.x << ", "
                          << "Rotation Center: (" << rotation_center.y << ", "
                          << "Rotation Center: (" << rotation_center.z << ")";
                
                MessageBuffer motionMsg = Protocol::createStringMessage(
                    motionInfo.str(), dataID);
                send(client_socket, &motionMsg, sizeof(MessageBuffer), 0);
            }
        }
        
        MessageBuffer endMsg = Protocol::createStringMessage("END", dataID);
        send(client_socket, &endMsg, sizeof(MessageBuffer), 0);
        
    } catch (const std::exception& e) {
        std::cerr << "Image processing error: " << e.what() << std::endl;
        MessageBuffer errorMsg = Protocol::createStringMessage(
            std::string("Processing error: ") + e.what(), dataID);
        send(client_socket, &errorMsg, sizeof(MessageBuffer), 0);
    }
}

void ArmorDetectionServer::handleTransformRequest(int client_socket, const MessageBuffer& msg) {
    try {
        double x, y, z, roll, pitch, yaw;
        Protocol::extractTransformData(msg, x, y, z, roll, pitch, yaw);
        
        // 直接使用提取的坐标和姿态数据
        cv::Point3f position(x, y, z);
        cv::Point3f orientation(roll, pitch, yaw);
        
        // 发送转换后的数据
        MessageBuffer response = Protocol::createTransformMessage(
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z,
            msg.DataID
        );
        
        if (send(client_socket, &response, sizeof(MessageBuffer), 0) < 0) {
            std::cerr << "Failed to send transform response" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error handling transform request: " << e.what() << std::endl;
    }
}

void ArmorDetectionServer::rotationMatrixToEulerAngles(const cv::Mat &R, double &roll, double &pitch, double &yaw) {
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    
    if (cv::norm(I, shouldBeIdentity) > 1e-6) {
        throw std::runtime_error("Matrix is not a rotation matrix");
    }

    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
    
    bool singular = sy < 1e-6;
    
    if (!singular) {
        roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = 0;
    }
    
    // 转换为度
    roll = roll * 180.0 / CV_PI;
    pitch = pitch * 180.0 / CV_PI;
    yaw = yaw * 180.0 / CV_PI;
}

void ArmorDetectionServer::cleanup() {
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
}

void ArmorDetectionServer::networkThread() {
    while (running_) {
        try {
            // 网络线程主要负责接收和处理客户端连接
            // 具体的客户端处理已经在 handleClient 中实现
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } catch (const std::exception& e) {
            if (running_) {
                std::cerr << "Network thread error: " << e.what() << std::endl;
            }
        }
    }
}

void ArmorDetectionServer::processingThread() {
    while (running_) {
        try {
            Task task;
            if (processing_queue_.try_pop(task)) {
                // 处理图像
                cv::Mat undistorted;
                cv::undistort(task.image, undistorted, camera_matrix_, dist_coeffs_);
                cv::Mat binary = preprocessImage(undistorted);
                
                auto light_bars = findLightBars(binary);
                auto armor_pairs = matchArmorPairs(light_bars);
                
                for (const auto& pair : armor_pairs) {
                    auto corners = getArmorCorners(pair);
                    cv::Mat rvec, tvec;
                    solveArmorPose(corners, rvec, tvec);
                    
                    cv::Point3f position(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                    
                    // 更新跟踪器
                    std::lock_guard<std::mutex> lock(tracker_mutex_);
                    auto& tracker = trackers_[task.dataID];
                    if (!tracker) {
                        tracker = std::make_unique<ArmorTracker>();
                    }
                    
                    cv::Point3f predicted_position = tracker->update(position, task.timestamp);
                    
                    // 将结果放入预测队列
                    Result result{task.dataID, position, predicted_position, task.timestamp};
                    prediction_queue_.push(result);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (const std::exception& e) {
            if (running_) {
                std::cerr << "Processing thread error: " << e.what() << std::endl;
            }
        }
    }
}

void ArmorDetectionServer::predictionThread() {
    while (running_) {
        try {
            Result result;
            if (prediction_queue_.try_pop(result)) {
                // 预测线程主要负责处理预测结果
                // 这里可以添加额外的预测逻辑
                output_queue_.push(result);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (const std::exception& e) {
            if (running_) {
                std::cerr << "Prediction thread error: " << e.what() << std::endl;
            }
        }
    }
}

int main(int argc, char** argv) {
    try {
        int port = 8080;
        size_t numThreads = 4;
        if (argc > 1) {
            port = std::stoi(argv[1]);
        }
        if (argc > 2) {
            numThreads = std::stoul(argv[2]);
        }

        ArmorDetectionServer server(port, numThreads);
        server.run();
    } catch (const std::exception& e) {
        std::cerr << "Server error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}