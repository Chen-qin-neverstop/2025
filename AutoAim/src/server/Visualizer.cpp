#include "Visualizer.h"
#include <fstream>
#include <iomanip>
#include <algorithm>

Visualizer::Visualizer() {
    // 初始化3D视图参数
    view_angle_x_ = 30.0f;
    view_angle_y_ = -45.0f;
    zoom_ = 1.0f;
}

Visualizer::~Visualizer() {
    // 清理资源
}

void Visualizer::addDataPoint(const RotationCenterData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_points_.push_back(data);
}

void Visualizer::showCurrentFrame(const cv::Mat& frame, const RotationCenterData& current_data) {
    cv::Mat display = frame.clone();
    
    // 显示当前数据
    std::string info = cv::format("Frame: %d  RC(raw): %.2f,%.2f,%.2f  RC(filtered): %.2f,%.2f,%.2f",
                                 current_data.frame,
                                 current_data.raw_center.x, current_data.raw_center.y, current_data.raw_center.z,
                                 current_data.filtered_center.x, current_data.filtered_center.y, current_data.filtered_center.z);
    
    cv::putText(display, info, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
    
    // 显示当前帧
    cv::imshow("Current Frame", display);
    
    // 更新3D视图
    update3DView();
}

void Visualizer::visualize3DTrajectory() {
    cv::Mat plot = create3DTrajectoryPlot();
    cv::imshow("3D Rotation Center Trajectory", plot);
    
    // 交互控制
    std::cout << "3D View Controls:\n";
    std::cout << "  W/S - Rotate up/down\n";
    std::cout << "  A/D - Rotate left/right\n";
    std::cout << "  Q/E - Zoom in/out\n";
    std::cout << "  ESC - Exit\n";
    
    while (true) {
        int key = cv::waitKey(30);
        
        if (key == 27) break; // ESC
        
        // 处理键盘输入
        switch (key) {
            case 'w': view_angle_x_ += 5.0f; break;
            case 's': view_angle_x_ -= 5.0f; break;
            case 'a': view_angle_y_ += 5.0f; break;
            case 'd': view_angle_y_ -= 5.0f; break;
            case 'q': zoom_ *= 1.1f; break;
            case 'e': zoom_ /= 1.1f; break;
        }
        
        // 更新视图
        update3DView();
    }
}

void Visualizer::update3DView() {
    cv::Mat plot = create3DTrajectoryPlot();
    cv::imshow("3D Rotation Center Trajectory", plot);
}

cv::Mat Visualizer::create3DTrajectoryPlot() {
    const int plot_size = 800;
    cv::Mat plot(plot_size, plot_size, CV_8UC3, cv::Scalar(240, 240, 240));
    
    if (data_points_.empty()) return plot;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 计算数据范围
    float min_x = data_points_[0].filtered_center.x;
    float max_x = min_x;
    float min_y = data_points_[0].filtered_center.y;
    float max_y = min_y;
    float min_z = data_points_[0].filtered_center.z;
    float max_z = min_z;
    
    for (const auto& data : data_points_) {
        min_x = std::min(min_x, data.filtered_center.x);
        max_x = std::max(max_x, data.filtered_center.x);
        min_y = std::min(min_y, data.filtered_center.y);
        max_y = std::max(max_y, data.filtered_center.y);
        min_z = std::min(min_z, data.filtered_center.z);
        max_z = std::max(max_z, data.filtered_center.z);
    }
    
    // 确保范围不为零
    float range = std::max({max_x - min_x, max_y - min_y, max_z - min_z, 0.1f});
    
    // 计算中心点
    cv::Point3f center((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2);
    
    // 设置3D视图参数
    float focal_length = 2.0f * plot_size / range * zoom_;
    cv::Point2f image_center(plot_size / 2, plot_size / 2);
    
    // 获取视图旋转矩阵
    cv::Mat rotation = getViewRotation();
    cv::Mat translation = (cv::Mat_<float>(3,1) << 0, 0, -2.5f * range * zoom_);
    
    // 绘制坐标轴
    cv::Point3f axis_x(center.x + range/2, center.y, center.z);
    cv::Point3f axis_y(center.x, center.y + range/2, center.z);
    cv::Point3f axis_z(center.x, center.y, center.z + range/2);
    
    cv::Point2f proj_origin = project3DPoint(center, rotation, translation, focal_length, image_center);
    cv::Point2f proj_x = project3DPoint(axis_x, rotation, translation, focal_length, image_center);
    cv::Point2f proj_y = project3DPoint(axis_y, rotation, translation, focal_length, image_center);
    cv::Point2f proj_z = project3DPoint(axis_z, rotation, translation, focal_length, image_center);
    
    cv::line(plot, proj_origin, proj_x, cv::Scalar(0, 0, 255), 2); // X轴(红色)
    cv::line(plot, proj_origin, proj_y, cv::Scalar(0, 255, 0), 2); // Y轴(绿色)
    cv::line(plot, proj_origin, proj_z, cv::Scalar(255, 0, 0), 2); // Z轴(蓝色)
    
    cv::putText(plot, "X", proj_x + cv::Point2f(5,5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
    cv::putText(plot, "Y", proj_y + cv::Point2f(5,5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
    cv::putText(plot, "Z", proj_z + cv::Point2f(5,5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0), 2);
    
    // 绘制轨迹
    for (size_t i = 1; i < data_points_.size(); i++) {
        const auto& prev = data_points_[i-1];
        const auto& curr = data_points_[i];
        
        cv::Point2f prev_raw = project3DPoint(prev.raw_center, rotation, translation, focal_length, image_center);
        cv::Point2f curr_raw = project3DPoint(curr.raw_center, rotation, translation, focal_length, image_center);
        
        cv::Point2f prev_filt = project3DPoint(prev.filtered_center, rotation, translation, focal_length, image_center);
        cv::Point2f curr_filt = project3DPoint(curr.filtered_center, rotation, translation, focal_length, image_center);
        
        // 绘制原始数据点(浅蓝色)
        cv::circle(plot, curr_raw, 2, cv::Scalar(255, 200, 100), -1);
        
        // 绘制滤波后轨迹(深红色)
        cv::line(plot, prev_filt, curr_filt, cv::Scalar(50, 50, 200), 2);
    }
    
    // 添加标题和视角信息
    std::string view_info = cv::format("View: X=%.0fdeg Y=%.0fdeg Zoom=%.1fx", view_angle_x_, view_angle_y_, zoom_);
    cv::putText(plot, "3D Rotation Center Trajectory", cv::Point(20, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(plot, view_info, cv::Point(20, 60), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    return plot;
}

cv::Point2f Visualizer::project3DPoint(const cv::Point3f& point, 
                                      const cv::Mat& rotation, 
                                      const cv::Mat& translation, 
                                      float focal_length, 
                                      const cv::Point2f& center) {
    cv::Mat point_mat = (cv::Mat_<float>(3,1) << point.x, point.y, point.z);
    cv::Mat rotated = rotation * point_mat + translation;
    
    float x = rotated.at<float>(0) * focal_length / rotated.at<float>(2) + center.x;
    float y = -rotated.at<float>(1) * focal_length / rotated.at<float>(2) + center.y;
    
    return cv::Point2f(x, y);
}

cv::Mat Visualizer::getViewRotation() {
    float x_rad = view_angle_x_ * CV_PI / 180.0f;
    float y_rad = view_angle_y_ * CV_PI / 180.0f;
    
    cv::Mat rot_x = (cv::Mat_<float>(3,3) << 
        1, 0, 0,
        0, cos(x_rad), -sin(x_rad),
        0, sin(x_rad), cos(x_rad));
    
    cv::Mat rot_y = (cv::Mat_<float>(3,3) << 
        cos(y_rad), 0, sin(y_rad),
        0, 1, 0,
        -sin(y_rad), 0, cos(y_rad));
    
    return rot_y * rot_x;
}

void Visualizer::saveToCSV(const std::string& filename) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }
    
    // 写入表头
    file << "Frame,Timestamp,Raw_X,Raw_Y,Raw_Z,Filtered_X,Filtered_Y,Filtered_Z\n";
    
    // 写入数据
    for (const auto& data : data_points_) {
        file << data.frame << ","
             << std::fixed << std::setprecision(6) << data.timestamp << ","
             << std::setprecision(4) << data.raw_center.x << ","
             << data.raw_center.y << "," << data.raw_center.z << ","
             << data.filtered_center.x << "," << data.filtered_center.y << ","
             << data.filtered_center.z << "\n";
    }
    
    file.close();
    std::cout << "Data saved to " << filename << std::endl;
}