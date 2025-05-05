#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <vector>
#include <opencv2/opencv.hpp>

// 消息类型枚举
enum MessageType : uint16_t {
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};

// 协议常量
const uint16_t START_SYMBOL = 0x0D00;
const uint16_t END_SYMBOL = 0x0721;
const size_t MAX_DATA_SIZE = 10218;

// 消息缓冲区结构
#pragma pack(push, 1)  // 确保结构体紧凑排列
struct MessageBuffer {
    uint16_t Start;             // 0x0D00
    uint16_t MessageType;
    uint32_t DataID;
    uint32_t DataTotalLength;
    uint32_t Offset;
    uint32_t DataLength;
    uint8_t Data[MAX_DATA_SIZE];
    uint16_t End;               // 0x0721
};
#pragma pack(pop)

// 协议工具类
class Protocol {
public:
    // 序列化消息
    static std::vector<uint8_t> serialize(const MessageBuffer& msg);
    
    // 反序列化消息
    static MessageBuffer deserialize(const uint8_t* data, size_t length);
    
    // 创建图像消息
    static std::vector<MessageBuffer> createImageMessage(const cv::Mat& image, uint32_t dataID);
    
    // 创建变换消息
    static MessageBuffer createTransformMessage(
        double x, double y, double z,
        double roll, double pitch, double yaw,
        uint32_t dataID);
    
    // 创建字符串消息
    static MessageBuffer createStringMessage(const std::string& str, uint32_t dataID);
    
    // 从消息中提取图像
    static cv::Mat extractImage(const std::vector<MessageBuffer>& messages);
    
    // 从消息中提取变换数据
    static void extractTransformData(const MessageBuffer& msg, 
                                   double& x, double& y, double& z,
                                   double& roll, double& pitch, double& yaw);
    
    // 验证消息完整性
    static bool validateMessage(const MessageBuffer& msg);
};

#endif // PROTOCOL_H