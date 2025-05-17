// 使用socket通信的server.cpp

#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"
#include "CoordinateTransformer.h"
#include "MotionEstimator.h"
#include "RotationCenterCalculator.h"
#include "KalmanFilter.h"
#include "ArmorTracker.h"
#include "DataBuffer.h"
#include <atomic>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

using namespace std;
using namespace cv;

// 通信协议定义
enum MessageType {
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};

const unsigned short START_SYMBOL = 0x0D00;
const unsigned short END_SYMBOL = 0x0721;

#pragma pack(push, 1)
struct MessageBuffer {
    unsigned short Start;
    unsigned short MessageType;
    unsigned int DataID;
    unsigned int DataTotalLength;
    unsigned int Offset;
    unsigned int DataLength;
    unsigned char Data[10218];
    unsigned short End;
};
#pragma pack(pop)

// 全局共享数据
ThreadSafeQueue<FrameData> frame_data_queue;
DoubleBuffer frame_double_buffer;
atomic<bool> stop_flag(false);
map<unsigned int, vector<unsigned char>> data_temp;  // 用于分片数据重组

// 相机参数
const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
    2065.0580175762857, 0.0, 658.9098266395495,
    0.0, 2086.886458338243, 531.5333174739342,
    0.0, 0.0, 1.0);

const Mat DIST_COEFFS = (Mat_<double>(5, 1) << 
    -0.051836613762195866, 0.29341513924119095, 
    0.001501183796729562, 0.0009386915104617738, 0.0);

// 装甲板尺寸
const float ARMOR_WIDTH = 135.0f;
const float LIGHT_BAR_LENGTH = 55.0f;

void drawRotationCenter(Mat& frame, const Point3f& center, 
                       const Mat& camera_matrix, const Mat& dist_coeffs, int color) {
    vector<Point3f> points{center};
    vector<Point2f> projected_points;
    
    projectPoints(points, Mat::zeros(3,1,CV_32F), Mat::zeros(3,1,CV_32F),
                 camera_matrix, dist_coeffs, projected_points);
    
    if (!projected_points.empty()) {
        if(color == 0){    // 绿色
            circle(frame, projected_points[0], 10, Scalar(0, 255, 0), 2);
        }
        else if(color == 1){   // 红色
            circle(frame, projected_points[0], 10, Scalar(0, 0, 255), 2);
        }
        putText(frame, "RC: " + to_string(center.x) + "," + to_string(center.y), 
               projected_points[0] + Point2f(15,0), 
               FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0), 1);
    }
}

unsigned char* receive_and_decode(MessageBuffer &message) {
    unsigned int offset = message.Offset;
    unsigned int length = message.DataLength;
    unsigned int total_length = message.DataTotalLength;
    unsigned int dataID = message.DataID;

    if (data_temp.find(dataID) == data_temp.end()) {
        data_temp[dataID] = vector<unsigned char>(total_length);
    }

    memcpy(data_temp[dataID].data() + offset, message.Data, length);

    if (offset + length >= total_length) {
        unsigned char* data = new unsigned char[total_length];
        memcpy(data, data_temp[dataID].data(), total_length);
        data_temp.erase(dataID);
        return data;
    } else {
        return nullptr;
    }
}

void processing_thread() {
    try {
        while (!stop_flag) {
            FrameData frame_data;
            if (frame_data_queue.try_pop(frame_data)) {
                Mat binary = preprocessImage(frame_data.frame);
                vector<RotatedRect> light_bars = findLightBars(binary);
                
                if (!frame_double_buffer.try_push(move(binary))) {
                    cerr << "Double buffer full, dropping frame" << endl;
                }
            }
            this_thread::sleep_for(1ms);
        }
    } catch (const exception& e) {
        cerr << "Processing thread error: " << e.what() << endl;
    }
}

void handleClient(int client_socket, RotationCenterKalmanFilter& rc_kalman, 
                 MotionEstimator& motion_estimator, RotationCenterCalculator& calculator) {
    MessageBuffer message;
    Point3f last_valid_rc;
    const float MAX_JUMP_DISTANCE = 0.3f;

    while (!stop_flag) {
        ssize_t bytes_received = recv(client_socket, &message, sizeof(MessageBuffer), 0);
        if (bytes_received <= 0) break;

        if (message.Start != START_SYMBOL || message.End != END_SYMBOL) {
            cerr << "Invalid message format" << endl;
            continue;
        }

        if (message.MessageType == IMAGE_MSG) {
            unsigned char* image_data = receive_and_decode(message);
            if (!image_data) continue;

            vector<unsigned char> buffer(image_data, image_data + message.DataTotalLength);
            Mat frame = imdecode(buffer, IMREAD_COLOR);
            delete[] image_data;

            if (frame.empty()) continue;

            // 图像处理流水线
            Mat binary = preprocessImage(frame);
            vector<RotatedRect> light_bars = findLightBars(binary);
            vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);

            if (armor_pairs.empty()) {
                try {
                    Point3f predicted_rc = rc_kalman.predict();
                    drawRotationCenter(frame, predicted_rc, CAMERA_MATRIX, DIST_COEFFS, 1);
                } catch (...) {}
                continue;
            }

            vector<Point2f> armor_corners = getArmorCorners(armor_pairs[0]);
            Mat rvec, tvec;
            solveArmorPose(armor_corners, rvec, tvec);
            Point3f current_pos(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

            Mat R;
            Rodrigues(rvec, R);
            double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
            bool singular = sy < 1e-6;
            double roll, pitch, yaw;
            if (!singular) {
                roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
                pitch = atan2(-R.at<double>(2,0), sy);
                yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
            }

            Pose camera_pose(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), yaw, pitch, roll);
            CoordinateTransformer transformer;
            Pose gimbal_pose = transformer.transformToTarget(camera_pose, "/Gimbal");

            MotionEstimator::MotionState state = motion_estimator.update(rvec, tvec);
            Point3f rotation_center = calculator.Calculate(gimbal_pose, state.linear_velocity, state.angular_velocity);

            try {
                Point3f filtered_rc = rc_kalman.update(rotation_center);
                if (norm(filtered_rc - last_valid_rc) > MAX_JUMP_DISTANCE) {
                    filtered_rc = rc_kalman.predict();
                }
                last_valid_rc = filtered_rc;
                drawRotationCenter(frame, filtered_rc, CAMERA_MATRIX, DIST_COEFFS, 1);
            } catch (...) {
                rc_kalman.init(rotation_center);
                last_valid_rc = rotation_center;
            }

            drawRotationCenter(frame, rotation_center, CAMERA_MATRIX, DIST_COEFFS, 0);
            drawDistanceInfo(frame, norm(tvec), armor_corners);
            imshow("Result", frame);
            waitKey(1);
        }
    }
    close(client_socket);
}

void socketServerThread(RotationCenterKalmanFilter& rc_kalman,
                       MotionEstimator& motion_estimator,
                       RotationCenterCalculator& calculator) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) return;

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    if (bind(server_fd, (sockaddr*)&address, sizeof(address)) < 0) {
        close(server_fd);
        return;
    }

    if (listen(server_fd, 3) < 0) {
        close(server_fd);
        return;
    }

    while (!stop_flag) {
        sockaddr_in client_address;
        socklen_t addrlen = sizeof(client_address);
        int client_socket = accept(server_fd, (sockaddr*)&client_address, &addrlen);
        if (client_socket < 0) continue;

        thread(handleClient, client_socket, ref(rc_kalman), 
               ref(motion_estimator), ref(calculator)).detach();
    }
    close(server_fd);
}

int main(int argc, char** argv) {
    RotationCenterKalmanFilter rc_kalman;
    MotionEstimator motion_estimator;
    RotationCenterCalculator calculator;
    Point3f last_valid_rc;

    // 启动处理线程
    thread processor(processing_thread);
    
    // 启动Socket服务器线程
    thread socket_thread(socketServerThread, ref(rc_kalman), 
                        ref(motion_estimator), ref(calculator));

    // 视频处理循环
    string video_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/2.mp4"; 
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "无法打开视频" << endl;
        stop_flag = true;
    }

    int frame_count = 0;
    Mat frame;

    while (!stop_flag && cap.isOpened()) {
        cap >> frame;
        if (frame.empty()) break;

        frame_count++;
        FrameData current_frame;
        current_frame.frame = frame.clone();
        current_frame.frame_count = frame_count;
        frame_data_queue.push(move(current_frame));

        if (frame_double_buffer.wait_for_data(10)) {
            auto processed_frames = frame_double_buffer.get_front_buffer();
            if (!processed_frames.empty()) {
                imshow("binary", processed_frames[0]);
            }
            frame_double_buffer.swap_buffers();
        }

        if (waitKey(30) == 27) {
            stop_flag = true;
            break;
        }
    }

    // 清理资源
    stop_flag = true;
    frame_data_queue.stop();
    frame_double_buffer.stop();
    processor.join();
    socket_thread.join();
    cap.release();
    destroyAllWindows();
    cout << "程序退出" << endl;
    return 0;
}