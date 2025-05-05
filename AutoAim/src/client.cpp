#include "Protocol.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class ArmorDetectionClient {
public:
    ArmorDetectionClient(const std::string& server_ip, int port) 
        : server_ip_(server_ip), port_(port) {}

    void sendImage(const cv::Mat& image) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            throw std::runtime_error("Socket creation failed");
        }

        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port_);

        if (inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
            throw std::runtime_error("Invalid address/ Address not supported");
        }

        if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            throw std::runtime_error("Connection failed");
        }

        try {
            // 创建并发送图像消息
            auto imageMessages = Protocol::createImageMessage(image, 1);
            for (const auto& msg : imageMessages) {
                send(sock, &msg, sizeof(MessageBuffer), 0);
                
                // 接收并处理服务器响应
                MessageBuffer response;
                ssize_t bytes_read = recv(sock, &response, sizeof(MessageBuffer), 0);
                
                if (bytes_read != sizeof(MessageBuffer)) {
                    throw std::runtime_error("Invalid response size");
                }
                
                if (response.MessageType == TRANSFORM) {
                    double x, y, z, roll, pitch, yaw;
                    Protocol::extractTransformData(response, x, y, z, roll, pitch, yaw);
                    
                    std::cout << "Detected armor at: "
                              << "X=" << x << " Y=" << y << " Z=" << z << " "
                              << "Roll=" << roll << " Pitch=" << pitch << " Yaw=" << yaw << std::endl;
                } else if (response.MessageType == STRING_MSG) {
                    std::string msgStr(response.Data, response.Data + response.DataLength);
                    std::cout << "Server message: " << msgStr << std::endl;
                    if (msgStr == "END") break;
                }
            }
        } catch (...) {
            close(sock);
            throw;
        }

        close(sock);
    }

private:
    std::string server_ip_;
    int port_;
};

int main() {
    try {
        ArmorDetectionClient client("127.0.0.1", 8080);
        
        // 加载测试图像
        cv::Mat image = cv::imread("test_image.jpg");
        if (image.empty()) {
            std::cerr << "Could not read the image" << std::endl;
            return -1;
        }
        
        client.sendImage(image);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}