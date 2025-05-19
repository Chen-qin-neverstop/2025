// 暂时停用
#pragma once

#include <cstdint>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>

namespace protocol {

// 消息类型枚举
enum class MessageType : uint8_t {
    VIDEO_FRAME = 0x01,    // 视频帧数据
    DETECTION_RESULT = 0x02, // 检测结果
    HEARTBEAT = 0x03,      // 心跳包
    ERROR = 0xFF           // 错误消息
};

// 消息头结构
struct MessageHeader {
    MessageType type;      // 消息类型
    uint32_t payload_size; // 负载大小
    uint32_t frame_id;     // 帧ID
    uint64_t timestamp;    // 时间戳
};

// 检测结果结构
struct DetectionResult {
    float x;              // 装甲板中心x坐标
    float y;              // 装甲板中心y坐标
    float width;          // 装甲板宽度
    float height;         // 装甲板高度
    float confidence;     // 置信度
    uint8_t armor_type;   // 装甲板类型
};

// 协议常量
const uint16_t START_SYMBOL = 0x0D00;
const uint16_t END_SYMBOL = 0x0721;
const size_t MAX_DATA_SIZE = 10218;

// 兼容旧代码的消息类型常量
constexpr uint16_t IMAGE_MSG = 1;
constexpr uint16_t TRANSFORM_REQUEST = 2;
constexpr uint16_t TRANSFORM = 3;
constexpr uint16_t STRING_MSG = 4;
constexpr uint16_t CORNERS_MSG = 5;

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

    // 新增：创建角点消息
    static MessageBuffer createCornersMessage(const std::vector<cv::Point2f>& corners, uint32_t dataID);
    
    // 新增：从消息中提取角点
    static std::vector<cv::Point2f> extractCorners(const MessageBuffer& msg);

    // 序列化视频帧
    static std::vector<uint8_t> serializeFrame(const cv::Mat& frame);

    // 反序列化视频帧
    static cv::Mat deserializeFrame(const std::vector<uint8_t>& data);

    // 序列化检测结果
    static std::vector<uint8_t> serializeDetectionResult(const DetectionResult& result);

    // 反序列化检测结果
    static DetectionResult deserializeDetectionResult(const std::vector<uint8_t>& data);
};

} // namespace protocol