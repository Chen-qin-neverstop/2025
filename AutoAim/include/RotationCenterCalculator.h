#ifndef ROTATION_CENTER_CALCULATOR_H
#define ROTATION_CENTER_CALCULATOR_H

#include <opencv2/opencv.hpp>
#include <vector>

class RotationCenterCalculator {
public:
    RotationCenterCalculator(float armor_width = 0.135f, float armor_height = 0.055f);
    cv::Point3f calculateRotationCenter(
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        float rotation_radius);

private:
    float armor_width_;
    float armor_height_;
    cv::Point3f calculateArmorCenterWorld(
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs);
    std::vector<cv::Point3f> getArmorObjectPoints() const;
};

#endif