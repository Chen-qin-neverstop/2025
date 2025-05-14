// #include "ArmorDetectionServer.h"
// #include "ImageProcess.h"
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <unistd.h>
// #include <arpa/inet.h>

// ArmorDetectionServer::ArmorDetectionServer(int port, int worker_threads) 
//     : port_(port), server_fd_(-1) {
    
//     // 初始化工作线程
//     for (int i = 0; i < worker_threads; ++i) {
//         worker_threads_.emplace_back(&ArmorDetectionServer::processingThread, this);
//     }
    
//     // 初始化预测线程
//     prediction_thread_ = std::thread(&ArmorDetectionServer::predictionThread, this);
// }

// ArmorDetectionServer::~ArmorDetectionServer() {
//     stop();
// }

// void ArmorDetectionServer::start() {
//     running_ = true;
    
//     // 创建socket
//     server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
//     if (server_fd_ < 0) {
//         throw std::runtime_error("Socket creation failed");
//     }
    
//     // 设置socket选项
//     int opt = 1;
//     if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
//         throw std::runtime_error("Setsockopt failed");
//     }
    
//     sockaddr_in address;
//     address.sin_family = AF_INET;
//     address.sin_addr.s_addr = INADDR_ANY;
//     address.sin_port = htons(port_);
    
//     // 绑定端口
//     if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
//         throw std::runtime_error("Bind failed");
//     }
    
//     // 开始监听
//     if (listen(server_fd_, 3) < 0) {
//         throw std::runtime_error("Listen failed");
//     }
    
//     std::cout << "Server started on port " << port_ << std::endl;
    
//     // 启动网络线程
//     network_thread_ = std::thread(&ArmorDetectionServer::networkThread, this);
// }

// void ArmorDetectionServer::stop() {
//     running_ = false;
    
//     // 关闭socket
//     if (server_fd_ != -1) {
//         close(server_fd_);
//         server_fd_ = -1;
//     }
    
//     // 等待线程结束
//     if (network_thread_.joinable()) {
//         network_thread_.join();
//     }
    
//     for (auto& thread : worker_threads_) {
//         if (thread.joinable()) {
//             thread.join();
//         }
//     }
    
//     if (prediction_thread_.joinable()) {
//         prediction_thread_.join();
//     }
// }

// void ArmorDetectionServer::networkThread() {
//     while (running_) {
//         sockaddr_in client_addr;
//         socklen_t client_len = sizeof(client_addr);
//         int client_socket = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        
//         if (client_socket < 0) {
//             if (running_) {
//                 std::cerr << "Accept failed" << std::endl;
//             }
//             continue;
//         }
        
//         // 接收消息
//         MessageBuffer msg;
//         ssize_t bytes_read = recv(client_socket, &msg, sizeof(MessageBuffer), 0);
        
//         if (bytes_read != sizeof(MessageBuffer)) {
//             close(client_socket);
//             continue;
//         }
        
//         if (!Protocol::validateMessage(msg)) {
//             close(client_socket);
//             continue;
//         }
        
//         if (msg.MessageType == IMAGE_MSG) {
//             // 接收完整图像数据
//             std::vector<MessageBuffer> messages;
//             messages.push_back(msg);
            
//             while (messages.back().Offset + messages.back().DataLength < messages.back().DataTotalLength) {
//                 MessageBuffer next_msg;
//                 bytes_read = recv(client_socket, &next_msg, sizeof(MessageBuffer), 0);
                
//                 if (bytes_read != sizeof(MessageBuffer)) {
//                     break;
//                 }
                
//                 messages.push_back(next_msg);
//             }
            
//             // 解码图像并加入处理队列
//             try {
//                 cv::Mat image = Protocol::extractImage(messages);
//                 double timestamp = cv::getTickCount() / cv::getTickFrequency();
//                 processing_queue_.push({msg.DataID, image, timestamp});
//             } catch (...) {
//                 std::cerr << "Failed to decode image" << std::endl;
//             }
//         }
        
//         close(client_socket);
//     }
// }

// void ArmorDetectionServer::processingThread() {
//     while (running_) {
//         Task task;
//         processing_queue_.wait_and_pop(task);
        
//         try {
//             // 图像处理
//             cv::Mat undistorted;
//             cv::undistort(task.image, undistorted, CAMERA_MATRIX, DIST_COEFFS);
//             cv::Mat binary = preprocessImage(undistorted);
            
//             auto light_bars = findLightBars(binary);
//             auto armor_pairs = matchArmorPairs(light_bars);
            
//             for (const auto& pair : armor_pairs) {
//                 auto corners = getArmorCorners(pair);
                
//                 cv::Mat rvec, tvec;
//                 solveArmorPose(corners, rvec, tvec);
                
//                 cv::Point3f position(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                
//                 // 发送到预测队列
//                 prediction_queue_.push({task.dataID, position, cv::Point3f(), task.timestamp});
//             }
//         } catch (...) {
//             std::cerr << "Image processing error" << std::endl;
//         }
//     }
// }

// void ArmorDetectionServer::predictionThread() {
//     while (running_) {
//         Result result;
//         prediction_queue_.wait_and_pop(result);
        
//         try {
//             std::unique_lock<std::mutex> lock(tracker_mutex_);
            
//             // 获取或创建跟踪器
//             auto& tracker = trackers_[result.dataID];
//             if (!tracker) {
//                 tracker = std::make_unique<ArmorTracker>(result.position, 0.1);
//             } else {
//                 double dt = result.timestamp - tracker->getLastUpdateTime();
//                 tracker->update(result.position, dt);
//             }
            
//             // 预测下一帧位置
//             result.predicted_position = tracker->predictNextPosition();
            
//             // 加入输出队列
//             output_queue_.push(result);
//         } catch (...) {
//             std::cerr << "Prediction error" << std::endl;
//         }
//     }
// }

#include "ArmorDetectionServer.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <csignal>
#include <iostream>

ArmorDetectionServer::ArmorDetectionServer(int port, int worker_threads) 
    : port_(port), server_fd_(-1) {
    
    // 初始化工作线程
    for (int i = 0; i < worker_threads; ++i) {
        worker_threads_.emplace_back(&ArmorDetectionServer::processingThread, this);
    }
    
    // 初始化预测线程
    prediction_thread_ = std::thread(&ArmorDetectionServer::predictionThread, this);
    
    threads_initialized_ = true;
}

ArmorDetectionServer::~ArmorDetectionServer() {
    stop();
}

void ArmorDetectionServer::start() {
    if (running_) return;

    running_ = true;
    
    // 创建socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        相机参数：ArmorDetectionServer构
        throw std::runtime_error("Socket creation failed");
    }
    
    // 设置socket选项
    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        close(server_fd_);
        throw std::runtime_error("Setsockopt failed");
    }
    
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);
    
    // 绑定端口
    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        close(server_fd_);
        throw std::runtime_error("Bind failed");
    }
    
    // 开始监听
    if (listen(server_fd_, 3) < 0) {
        close(server_fd_);
        throw std::runtime_error("Listen failed");
    }
    
    std::cout << "Server started on port " << port_ << std::endl;
    
    // 启动网络线程
    network_thread_ = std::thread(&ArmorDetectionServer::networkThread, this);
}

void ArmorDetectionServer::stop() {
    if (!running_) return;

    running_ = false;
    
    // 唤醒所有可能等待的线程
    processing_queue_.shutdown();
    prediction_queue_.shutdown();
    output_queue_.shutdown();
    
    // 关闭socket会中断accept调用
    if (server_fd_ != -1) {
        shutdown(server_fd_, SHUT_RDWR);
        close(server_fd_);
        server_fd_ = -1;
    }
    
    cleanup();
}

bool ArmorDetectionServer::isRunning() const {
    return running_;
}

void ArmorDetectionServer::cleanup() {
    // 等待网络线程结束
    if (network_thread_.joinable()) {
        network_thread_.join();
    }
    
    // 等待工作线程结束
    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    // 等待预测线程结束
    if (prediction_thread_.joinable()) {
        prediction_thread_.join();
    }
    
    // 清空队列
    processing_queue_.clear();
    prediction_queue_.clear();
    output_queue_.clear();
    
    // 清空跟踪器
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    trackers_.clear();
}

void ArmorDetectionServer::networkThread() {
    while (running_) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_socket < 0) {
            if (running_) {
                std::cerr << "Accept failed: " << strerror(errno) << std::endl;
            }
            continue;
        }
        
        // 设置接收超时
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        try {
            // 接收消息
            MessageBuffer msg;
            ssize_t bytes_read = recv(client_socket, &msg, sizeof(MessageBuffer), MSG_WAITALL);
            
            if (bytes_read != sizeof(MessageBuffer)) {
                close(client_socket);
                continue;
            }
            
            if (!Protocol::validateMessage(msg)) {
                close(client_socket);
                continue;
            }
            
            if (msg.MessageType == IMAGE_MSG) {
                // 接收完整图像数据
                std::vector<MessageBuffer> messages;
                messages.push_back(msg);
                
                while (messages.back().Offset + messages.back().DataLength < messages.back().DataTotalLength) {
                    MessageBuffer next_msg;
                    bytes_read = recv(client_socket, &next_msg, sizeof(MessageBuffer), MSG_WAITALL);
                    
                    if (bytes_read != sizeof(MessageBuffer)) {
                        break;
                    }
                    
                    messages.push_back(next_msg);
                }
                
                // 解码图像并加入处理队列
                try {
                    cv::Mat image = Protocol::extractImage(messages);
                    double timestamp = cv::getTickCount() / cv::getTickFrequency();
                    processing_queue_.push({msg.DataID, image, timestamp});
                } catch (const std::exception& e) {
                    std::cerr << "Failed to decode image: " << e.what() << std::endl;
                }
            }
        } catch (...) {
            std::cerr << "Error processing client request" << std::endl;
        }
        
        close(client_socket);
    }
}

void ArmorDetectionServer::processingThread() {
    while (running_) {
        Task task;
        if (!processing_queue_.wait_and_pop(task)) {
            break;  // 队列已关闭
        }
        
        try {
            // 图像处理
            cv::Mat undistorted;
            cv::undistort(task.image, undistorted, CAMERA_MATRIX, DIST_COEFFS);
            cv::Mat binary = preprocessImage(undistorted);
            
            auto light_bars = findLightBars(binary);
            auto armor_pairs = matchArmorPairs(light_bars);
            
            for (const auto& pair : armor_pairs) {
                auto corners = getArmorCorners(pair);
                
                cv::Mat rvec, tvec;
                solveArmorPose(corners, rvec, tvec);
                
                cv::Point3f position(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                
                // 发送到预测队列
                prediction_queue_.push({task.dataID, position, cv::Point3f(), task.timestamp});
            }
        } catch (const std::exception& e) {
            std::cerr << "Image processing error: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown image processing error" << std::endl;
        }
    }
}

void ArmorDetectionServer::predictionThread() {
    while (running_) {
        Result result;
        if (!prediction_queue_.wait_and_pop(result)) {
            break;  // 队列已关闭
        }
        
        try {
            std::unique_lock<std::mutex> lock(tracker_mutex_);
            
            // 获取或创建跟踪器
            auto& tracker = trackers_[result.dataID];
            if (!tracker) {
                tracker = std::make_unique<ArmorTracker>(result.position, 0.1);
            } else {
                double dt = result.timestamp - tracker->getLastUpdateTime();
                tracker->update(result.position, dt);
            }
            
            // 预测下一帧位置
            result.predicted_position = tracker->predictNextPosition();
            
            // 加入输出队列
            output_queue_.push(result);
        } catch (const std::exception& e) {
            std::cerr << "Prediction error: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown prediction error" << std::endl;
        }
    }
}