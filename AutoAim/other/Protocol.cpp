// 暂时停用
#include "Protocol.h"
#include <vector>
#include <cstring>
#include <stdexcept>
#include <sstream>

using namespace protocol;

std::vector<uint8_t> Protocol::serialize(const MessageBuffer& msg) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&msg);
    return std::vector<uint8_t>(ptr, ptr + sizeof(MessageBuffer));
}

MessageBuffer Protocol::deserialize(const uint8_t* data, size_t length) {
    if (length != sizeof(MessageBuffer)) {
        throw std::runtime_error("Invalid message length");
    }
    
    MessageBuffer msg;
    memcpy(&msg, data, sizeof(MessageBuffer));
    
    if (!validateMessage(msg)) {
        throw std::runtime_error("Invalid message format");
    }
    
    return msg;
}

std::vector<MessageBuffer> Protocol::createImageMessage(const cv::Mat& image, uint32_t dataID) {
    std::vector<uint8_t> buffer;
    cv::imencode(".jpg", image, buffer);
    
    std::vector<MessageBuffer> messages;
    const size_t totalLength = buffer.size();
    size_t offset = 0;
    
    while (offset < totalLength) {
        MessageBuffer msg;
        msg.Start = START_SYMBOL;
        msg.MessageType = IMAGE_MSG;
        msg.DataID = dataID;
        msg.DataTotalLength = static_cast<uint32_t>(totalLength);
        msg.Offset = static_cast<uint32_t>(offset);
        
        const size_t chunkSize = std::min(MAX_DATA_SIZE, totalLength - offset);
        msg.DataLength = static_cast<uint32_t>(chunkSize);
        memcpy(msg.Data, buffer.data() + offset, chunkSize);
        
        msg.End = END_SYMBOL;
        messages.push_back(msg);
        
        offset += chunkSize;
    }
    
    return messages;
}

MessageBuffer Protocol::createTransformMessage(
    double x, double y, double z,
    double roll, double pitch, double yaw,
    uint32_t dataID) {
    
    MessageBuffer msg;
    msg.Start = START_SYMBOL;
    msg.MessageType = TRANSFORM;
    msg.DataID = dataID;
    msg.DataTotalLength = 6 * sizeof(double);
    msg.Offset = 0;
    msg.DataLength = 6 * sizeof(double);
    
    double* dataPtr = reinterpret_cast<double*>(msg.Data);
    dataPtr[0] = x;
    dataPtr[1] = y;
    dataPtr[2] = z;
    dataPtr[3] = roll;
    dataPtr[4] = pitch;
    dataPtr[5] = yaw;
    
    msg.End = END_SYMBOL;
    return msg;
}

MessageBuffer Protocol::createStringMessage(const std::string& str, uint32_t dataID) {
    MessageBuffer msg;
    msg.Start = START_SYMBOL;
    msg.MessageType = STRING_MSG;
    msg.DataID = dataID;
    msg.DataTotalLength = static_cast<uint32_t>(str.size());
    msg.Offset = 0;
    msg.DataLength = static_cast<uint32_t>(std::min(str.size(), MAX_DATA_SIZE));
    
    memcpy(msg.Data, str.data(), msg.DataLength);
    msg.End = END_SYMBOL;
    
    return msg;
}

cv::Mat Protocol::extractImage(const std::vector<MessageBuffer>& messages) {
    if (messages.empty()) {
        throw std::runtime_error("No messages to extract image from");
    }
    
    const uint32_t totalLength = messages[0].DataTotalLength;
    std::vector<uint8_t> buffer(totalLength);
    
    for (const auto& msg : messages) {
        if (msg.MessageType != IMAGE_MSG) {
            throw std::runtime_error("Invalid message type for image extraction");
        }
        
        if (msg.Offset + msg.DataLength > totalLength) {
            throw std::runtime_error("Invalid message offset/length");
        }
        
        memcpy(buffer.data() + msg.Offset, msg.Data, msg.DataLength);
    }
    
    return cv::imdecode(buffer, cv::IMREAD_COLOR);
}

void Protocol::extractTransformData(const MessageBuffer& msg, 
                                  double& x, double& y, double& z,
                                  double& roll, double& pitch, double& yaw) {
    if (msg.MessageType != TRANSFORM) {
        throw std::runtime_error("Invalid message type for transform extraction");
    }
    
    if (msg.DataLength != 6 * sizeof(double)) {
        throw std::runtime_error("Invalid data length for transform data");
    }
    
    const double* dataPtr = reinterpret_cast<const double*>(msg.Data);
    x = dataPtr[0];
    y = dataPtr[1];
    z = dataPtr[2];
    roll = dataPtr[3];
    pitch = dataPtr[4];
    yaw = dataPtr[5];
}

bool Protocol::validateMessage(const MessageBuffer& msg) {
    return msg.Start == START_SYMBOL && 
           msg.End == END_SYMBOL &&
           msg.DataLength <= MAX_DATA_SIZE;
}

MessageBuffer Protocol::createCornersMessage(const std::vector<cv::Point2f>& corners, uint32_t dataID) {
    MessageBuffer msg;
    msg.MessageType = CORNERS_MSG;
    msg.DataID = dataID;
    
    // 每个角点有x,y两个float，共4个角点
    const size_t dataSize = corners.size() * sizeof(float) * 2;
    msg.DataLength = dataSize;
    msg.DataTotalLength = dataSize;
    msg.Offset = 0;
    
    // 将角点数据打包到Data中
    float* data = reinterpret_cast<float*>(msg.Data);
    for (const auto& corner : corners) {
        *data++ = corner.x;
        *data++ = corner.y;
    }
    
    return msg;
}

std::vector<cv::Point2f> Protocol::extractCorners(const MessageBuffer& msg) {
    if (msg.MessageType != CORNERS_MSG) {
        throw std::runtime_error("Invalid message type for corners extraction");
    }
    
    const size_t numCorners = msg.DataLength / (sizeof(float) * 2);
    std::vector<cv::Point2f> corners;
    corners.reserve(numCorners);
    
    const float* data = reinterpret_cast<const float*>(msg.Data);
    for (size_t i = 0; i < numCorners; ++i) {
        float x = *data++;
        float y = *data++;
        corners.emplace_back(x, y);
    }
    
    return corners;
}