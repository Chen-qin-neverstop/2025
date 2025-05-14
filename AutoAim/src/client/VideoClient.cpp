#include "VideoClient.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>

using namespace protocol;

VideoClient::VideoClient() {
    main_thread_id_ = std::this_thread::get_id();
}

VideoClient::~VideoClient() {
    std::cout << "[VideoClient] Destructor called from thread: " << std::this_thread::get_id() << std::endl;
    if (std::this_thread::get_id() != main_thread_id_) {
        std::cerr << "[VideoClient] Destructor called from non-main thread, skipping cleanup." << std::endl;
        return;
    }
    stop();
    cleanup();
}

void VideoClient::stop() {
    std::cout << "[VideoClient] stop() called from thread: " << std::this_thread::get_id() << std::endl;
    if (std::this_thread::get_id() != main_thread_id_) {
        std::cerr << "[VideoClient] stop() called from non-main thread, skipping." << std::endl;
        return;
    }
    running_ = false;
    cv_.notify_all();
}

void VideoClient::cleanup() {
    std::cout << "[VideoClient] cleanup() called from thread: " << std::this_thread::get_id() << std::endl;
    if (std::this_thread::get_id() != main_thread_id_) {
        std::cerr << "[VideoClient] cleanup() called from non-main thread, skipping join to avoid deadlock." << std::endl;
        return;
    }
    if (cleaned_up_.exchange(true)) {
        std::cerr << "[VideoClient] cleanup() already called, skipping." << std::endl;
        return;
    }
    std::cout << "Cleaning up resources..." << std::endl;

    if (threads_initialized_) {
        std::thread::id this_id = std::this_thread::get_id();
        try {
            if (video_thread_.joinable() && video_thread_.get_id() != this_id) {
                video_thread_.join();
            }
        } catch (const std::system_error& e) {
            std::cerr << "Warning: join video_thread_ failed: " << e.what() << std::endl;
        }
        try {
            if (transform_thread_.joinable() && transform_thread_.get_id() != this_id) {
                transform_thread_.join();
            }
        } catch (const std::system_error& e) {
            std::cerr << "Warning: join transform_thread_ failed: " << e.what() << std::endl;
        }
        try {
            if (predict_thread_.joinable() && predict_thread_.get_id() != this_id) {
                predict_thread_.join();
            }
        } catch (const std::system_error& e) {
            std::cerr << "Warning: join predict_thread_ failed: " << e.what() << std::endl;
        }
    }
    std::cout << "Cleanup completed" << std::endl;
}

void VideoClient::start() {
    if (!running_) {
        running_ = true;
        video_thread_ = std::thread(&VideoClient::videoCaptureThread, this);
        transform_thread_ = std::thread(&VideoClient::transformThread, this);
        predict_thread_ = std::thread(&VideoClient::predictThread, this);
        threads_initialized_ = true;
    }
}

void VideoClient::run() {
    start();
    // 主线程（即 run 调用者）不执行视频捕获，由 videoCaptureThread 线程执行
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待线程启动
    std::cout << "VideoClient::run() finished, exiting." << std::endl;
}

void VideoClient::videoCaptureThread() {
    std::cout << "[VideoClient] videoCaptureThread started from thread: " << std::this_thread::get_id() << std::endl;
    cv::VideoCapture cap;
    if (!cap.open("/home/chen/视频/video/videos/2.mp4")) {
        std::cout << "Failed to open video file, trying camera..." << std::endl;
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
         std::cerr << "Failed to open any video source" << std::endl;
         throw std::runtime_error("Failed open video source");
    }
    double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = cap.get(cv::CAP_PROP_FPS);
    double total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    if (actual_fps > 30) {
         std::cout << "Warning: Video FPS (" << actual_fps << ") is too high, limiting to 30 FPS" << std::endl;
         actual_fps = 30;
    }
    std::cout << "Video properties: Resolution: " << actual_width << "x" << actual_height << " FPS: " << actual_fps << (total_frames > 0 ? (" Total frames: " + std::to_string(total_frames)) : "") << std::endl;
    uint32_t frame_id = 0;
    int failed_frames = 0;
    double last_frame_time = cv::getTickCount() / cv::getTickFrequency();
    const double min_frame_interval = 1.0 / actual_fps;
    while (running_) {
         try {
             cv::Mat frame;
             if (!cap.read(frame)) {
                 if (total_frames > 0) {
                     std::cout << "End of video, restarting..." << std::endl;
                     cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                     continue;
                 } else {
                     std::cerr << "Failed capture frame " << ++failed_frames << std::endl;
                     if (failed_frames > 10) {
                         std::cerr << "Too many failed frames, stopping capture" << std::endl;
                         break;
                     }
                     continue;
                 }
             }
             failed_frames = 0;
             if (frame.empty()) {
                 std::cerr << "Received empty frame" << std::endl;
                 continue;
             }
             double current_time = cv::getTickCount() / cv::getTickFrequency();
             double elapsed = current_time - last_frame_time;
             if (elapsed < min_frame_interval) {
                 std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((min_frame_interval - elapsed) * 1000)));
             }
             last_frame_time = cv::getTickCount() / cv::getTickFrequency();
             // 图像处理：二值化、轮廓检测、最小外接矩形，得到装甲板角点（即旋转中心）
             cv::Mat gray, binary;
             cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
             cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
             std::vector<std::vector<cv::Point>> contours;
             cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
             std::vector<cv::Point2f> centers;
             for (const auto& contour : contours) {
                 if (contour.size() < 5) continue; // 忽略太小的轮廓
                 cv::RotatedRect rect = cv::minAreaRect(contour);
                 centers.push_back(rect.center);
             }
             // 将装甲板角点（即旋转中心）存入 armor_corners_，并通知预测线程
             {
                 std::lock_guard<std::mutex> lock(mutex_);
                 armor_corners_ = std::move(centers);
                 has_new_corners_ = true;
             }
             cv_.notify_one();
             ++frame_id;
         } catch (const std::exception& e) {
             std::cerr << "Error in videoCaptureThread: " << e.what() << std::endl;
             if (running_) break;
         }
    }
    std::cout << "Cleaning up video capture..." << std::endl;
    cap.release();
    std::cout << "Video capture cleanup completed" << std::endl;
}

void VideoClient::transformThread() {
    std::cout << "[VideoClient] transformThread started from thread: " << std::this_thread::get_id() << std::endl;
    while (running_) {
         std::unique_lock<std::mutex> lock(mutex_);
         cv_.wait(lock, [this] { return !running_ || has_new_corners_; });
         if (!running_) break;
         if (has_new_corners_) {
             std::vector<cv::Point2f> corners(armor_corners_);
             has_new_corners_ = false;
             lock.unlock();
             // 这里利用仿射变换或PnP将装甲板角点（即旋转中心）转换为世界坐标，并输出文本格式
             for (const auto& pt : corners) {
                 std::cout << "Transform: Armor Rotation Center (world): (" << std::fixed << std::setprecision(2) << pt.x << ", " << pt.y << ")" << std::endl;
             }
         }
    }
}

void VideoClient::predictThread() {
    std::cout << "[VideoClient] predictThread started from thread: " << std::this_thread::get_id() << std::endl;
    while (running_) {
         std::unique_lock<std::mutex> lock(mutex_);
         cv_.wait(lock, [this] { return !running_ || has_new_corners_; });
         if (!running_) break;
         if (has_new_corners_) {
             std::vector<cv::Point2f> corners(armor_corners_);
             has_new_corners_ = false;
             lock.unlock();
             // 这里根据 armor_corners_ 预测装甲板旋转中心，并输出文本格式
             for (const auto& pt : corners) {
                 std::cout << "Predict: Armor Rotation Center (predicted): (" << std::fixed << std::setprecision(2) << pt.x << ", " << pt.y << ")" << std::endl;
             }
         }
    }
}

int main(int argc, char** argv) {
    try {
         VideoClient client;
         client.run();
    } catch (const std::exception& e) {
         std::cerr << "Client error: " << e.what() << std::endl;
         return 1;
    }
    return 0;
} 