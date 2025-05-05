#include "Protocol.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <server_ip> <port> [image_path]" << std::endl;
        return 1;
    }

    const char* server_ip = argv[1];
    int port = std::stoi(argv[2]);
    std::string image_path = (argc > 3) ? argv[3] : "assets/test_images/armor_sample1.jpg";

    // 创建socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return 1;
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // 转换IP地址
    if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return 1;
    }

    // 连接服务器
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection failed" << std::endl;
        return 1;
    }

    try {
        // 读取图像
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "Could not read the image: " << image_path << std::endl;
            return 1;
        }

        // 创建并发送图像消息
        auto imageMessages = Protocol::createImageMessage(image, 1);
        for (const auto& msg : imageMessages) {
            send(sock, &msg, sizeof(MessageBuffer), 0);
            
            // 接收服务器响应
            MessageBuffer response;
            ssize_t bytes_read = recv(sock, &response, sizeof(MessageBuffer), 0);
            
            if (bytes_read != sizeof(MessageBuffer)) {
                std::cerr << "Invalid response size" << std::endl;
                break;
            }
            
            if (response.MessageType == TRANSFORM) {
                double x, y, z, roll, pitch, yaw;
                Protocol::extractTransformData(response, x, y, z, roll, pitch, yaw);
                
                std::cout << "Armor Pose - "
                          << "Position: (" << x << ", " << y << ", " << z << ") | "
                          << "Orientation: (" << roll << ", " << pitch << ", " << yaw << ")" 
                          << std::endl;
            } 
            else if (response.MessageType == STRING_MSG) {
                std::string msgStr(response.Data, response.Data + response.DataLength);
                std::cout << "Server: " << msgStr << std::endl;
                if (msgStr == "END") break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    close(sock);
    return 0;
}