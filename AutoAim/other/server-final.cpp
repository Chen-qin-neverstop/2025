#include "Protocol.h"
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

class ArmorDetectionServer {
public:
    ArmorDetectionServer(int port) : port_(port), 
                                   rotation_center_calculator_(0.135f, 0.055f) {
        // 初始化相机参数
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            2065.0580175762857, 0.0, 658.9098266395495,
            0.0, 2086.886458338243, 531.5333174739342,
            0.0, 0.0, 1.0;)

        dist_coeffs_ = (cv::Mat_<double>(5, 1) << 
            -0.051836613762195866, 0.29341513924119095, 
            0.001501183796729562, 0.0009386915104617738, 0.0;)
    }

    void run() {
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

        std::cout << "Server listening on port " << port_ << std::endl;

        while (true) {
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_socket = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            
            if (client_socket < 0) {
                std::cerr << "Accept failed" << std::endl;
                continue;
            }

            handleClient(client_socket);
            close(client_socket);
        }
    }

private:
    void handleClient(int client_socket) {
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

    void handleImageMessage(int client_socket, const MessageBuffer& firstMsg) {
        std::vector<MessageBuffer> messages;
        messages.push_back(firstMsg);

        // 接收完整图像数据
        while (messages.back().Offset + messages.back().DataLength < messages.back().DataTotalLength) {
            MessageBuffer nextMsg;
            ssize_t bytes_read = recv(client_socket, &nextMsg, sizeof(MessageBuffer), 0);
            
            if (bytes_read != sizeof(MessageBuffer)) {
                throw std::runtime_error("Failed to receive next image chunk");
            }
            
            messages.push_back(nextMsg);
        }

        // 提取并处理图像
        cv::Mat image = Protocol::extractImage(messages);
        processImageAndSendResult(client_socket, image, firstMsg.DataID);
    }

    void processImageAndSendResult(int client_socket, const cv::Mat& image, uint32_t dataID) {
        try {
            // 图像预处理
            cv::Mat undistorted;
            cv::undistort(image, undistorted, camera_matrix_, dist_coeffs_);
            cv::Mat binary = preprocessImage(undistorted);
            
            // 检测装甲板
            auto light_bars = findLightBars(binary);
            auto armor_pairs = matchArmorPairs(light_bars);
            
            // 处理每个检测到的装甲板
            for (const auto& pair : armor_pairs) {
                auto corners = getArmorCorners(pair);
                
                cv::Mat rvec, tvec;
                solveArmorPose(corners, rvec, tvec);
                
                // 转换为旋转矩阵
                cv::Mat rot_mat;
                cv::Rodrigues(rvec, rot_mat);
                
                // 转换为欧拉角
                double roll, pitch, yaw;
                rotationMatrixToEulerAngles(rot_mat, roll, pitch, yaw);
                
                // 创建变换消息并发送
                MessageBuffer transformMsg = Protocol::createTransformMessage(
                    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                    roll, pitch, yaw, dataID);
                send(client_socket, &transformMsg, sizeof(MessageBuffer), 0);
                
                // 更新运动状态估计
                auto& estimator = motion_estimators_[dataID];
                auto motion_state = estimator.update(rvec, tvec);
                
                // 计算并发送运动状态信息
                if (motion_state.rotation_radius > 0) {
                    // 计算旋转中心
                    cv::Point3f rotation_center = rotation_center_calculator_.calculateRotationCenter(
                        corners, camera_matrix_, dist_coeffs_, motion_state.rotation_radius);
                    
                    // 存储旋转中心
                    rotation_centers_[dataID] = rotation_center;
                    
                    // 发送运动状态信息
                    std::stringstream motionInfo;
                    motionInfo << "Motion State - "
                              << "Linear Vel: (" << motion_state.linear_velocity.x << ", "
                              << motion_state.linear_velocity.y << ", "
                              << motion_state.linear_velocity.z << ") m/s | "
                              << "Angular Vel: (" << motion_state.angular_velocity.x << ", "
                              << motion_state.angular_velocity.y << ", "
                              << motion_state.angular_velocity.z << ") rad/s | "
                              << "Rotation Radius: " << motion_state.rotation_radius << " m | "
                              << "Rotation Center: (" << rotation_center.x << ", "
                              << rotation_center.y << ", " << rotation_center.z << ")";
                    
                    MessageBuffer motionMsg = Protocol::createStringMessage(
                        motionInfo.str(), dataID);
                    send(client_socket, &motionMsg, sizeof(MessageBuffer), 0);
                }
            }
            
            // 发送结束标志
            MessageBuffer endMsg = Protocol::createStringMessage("END", dataID);
            send(client_socket, &endMsg, sizeof(MessageBuffer), 0);
            
        } catch (const std::exception& e) {
            std::cerr << "Image processing error: " << e.what() << std::endl;
            MessageBuffer errorMsg = Protocol::createStringMessage(
                std::string("Processing error: ") + e.what(), dataID);
            send(client_socket, &errorMsg, sizeof(MessageBuffer), 0);
        }
    }

    void handleTransformRequest(int client_socket, const MessageBuffer& msg) {
        try {
            // 提取变换数据
            double x, y, z, roll, pitch, yaw;
            Protocol::extractTransformData(msg, x, y, z, roll, pitch, yaw);
            
            // 坐标转换
            CoordinateTransformer transformer;
            Pose camera_pose(x, y, z, roll, pitch, yaw);
            
            // 转换到不同坐标系
            Pose gimbal_pose = transformer.transformToTarget(camera_pose, "/Gimbal");
            Pose odom_pose = transformer.transformToTarget(camera_pose, "/Odom");
            
            // 发送转换结果
            auto gimbal_pos = gimbal_pose.getPosition();
            auto gimbal_orient = gimbal_pose.getOrientation();
            MessageBuffer gimbalMsg = Protocol::createTransformMessage(
                gimbal_pos[0], gimbal_pos[1], gimbal_pos[2],
                gimbal_orient[0], gimbal_orient[1], gimbal_orient[2],
                msg.DataID);
            send(client_socket, &gimbalMsg, sizeof(MessageBuffer), 0);
            
            auto odom_pos = odom_pose.getPosition();
            auto odom_orient = odom_pose.getOrientation();
            MessageBuffer odomMsg = Protocol::createTransformMessage(
                odom_pos[0], odom_pos[1], odom_pos[2],
                odom_orient[0], odom_orient[1], odom_orient[2],
                msg.DataID);
            send(client_socket, &odomMsg, sizeof(MessageBuffer), 0);
            
        } catch (const std::exception& e) {
            std::cerr << "Transform error: " << e.what() << std::endl;
            MessageBuffer errorMsg = Protocol::createStringMessage(
                std::string("Transform error: ") + e.what(), msg.DataID);
            send(client_socket, &errorMsg, sizeof(MessageBuffer), 0);
        }
    }

    void rotationMatrixToEulerAngles(const cv::Mat &R, double &roll, double &pitch, double &yaw) {
        // 检查是否为旋转矩阵
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

private:
    int port_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    RotationCenterCalculator rotation_center_calculator_;
    std::unordered_map<uint32_t, MotionEstimator> motion_estimators_;
    std::unordered_map<uint32_t, cv::Point3f> rotation_centers_;
};

int main(int argc, char** argv) {
    try {
        int port = 8080;
        if (argc > 1) {
            port = std::stoi(argv[1]);
        }

        ArmorDetectionServer server(port);
        server.run();
    } catch (const std::exception& e) {
        std::cerr << "Server error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}