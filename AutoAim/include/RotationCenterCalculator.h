#ifndef ROTATION_CENTER_CALCULATOR_H
#define ROTATION_CENTER_CALCULATOR_H

#include <opencv2/opencv.hpp>
#include <vector>

class RotationCenterCalculator {
public:
    // 设置装甲板在世界坐标系中的标准尺寸
    RotationCenterCalculator(float armor_width = 0.135f, float armor_height = 0.055f)
        : armor_width_(armor_width), armor_height_(armor_height) {}

    // 计算旋转中心
    cv::Point3f calculateRotationCenter(
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        float rotation_radius);

private:
    float armor_width_;
    float armor_height_;

    // 计算装甲板中心在世界坐标系中的位置
    cv::Point3f calculateArmorCenterWorld(
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs);

    // 获取装甲板在世界坐标系中的标准角点
    std::vector<cv::Point3f> getArmorObjectPoints() const;
};

#endif // ROTATION_CENTER_CALCULATOR_H