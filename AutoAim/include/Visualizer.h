#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <mutex>

struct RotationCenterData {
    int frame;
    cv::Point3f raw_center;
    cv::Point3f filtered_center;
    double timestamp;
};

class Visualizer {
public:
    Visualizer();
    ~Visualizer();

    // 添加数据点
    void addDataPoint(const RotationCenterData& data);
    
    // 实时显示当前帧和轨迹
    void showCurrentFrame(const cv::Mat& frame, const RotationCenterData& current_data);
    
    // 3D轨迹可视化
    void visualize3DTrajectory();
    
    // 保存数据到CSV
    void saveToCSV(const std::string& filename);

    // 更新3D视图
    void update3DView();

private:
    std::vector<RotationCenterData> data_points_;
    std::mutex data_mutex_;
    
    // 3D可视化参数
    float view_angle_x_ = 30.0f;
    float view_angle_y_ = -45.0f;
    float zoom_ = 1.0f;
    
    // 创建3D轨迹图
    cv::Mat create3DTrajectoryPlot();
    
    // 将3D点投影到2D图像
    cv::Point2f project3DPoint(const cv::Point3f& point, 
                              const cv::Mat& rotation, 
                              const cv::Mat& translation, 
                              float focal_length, 
                              const cv::Point2f& center);
    
    // 获取3D视图的旋转矩阵
    cv::Mat getViewRotation();
};

#endif // VISUALIZER_H