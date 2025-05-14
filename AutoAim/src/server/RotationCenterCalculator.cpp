#include "RotationCenterCalculator.h"
#include <iostream>

RotationCenterCalculator::RotationCenterCalculator(float armor_width, float armor_height)
    : armor_width_(armor_width), armor_height_(armor_height) {}

cv::Point3f RotationCenterCalculator::calculateRotationCenter(
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    float rotation_radius) {
    
    // 1. 计算装甲板中心
    cv::Point3f armor_center = calculateArmorCenterWorld(image_points, camera_matrix, dist_coeffs);
    
    // 2. 计算装甲板朝向
    cv::Mat rvec, tvec;
    cv::solvePnP(getArmorObjectPoints(), image_points, camera_matrix, dist_coeffs, rvec, tvec);
    
    cv::Mat rot_mat;
    cv::Rodrigues(rvec, rot_mat);
    
    cv::Point3f armor_direction(
        rot_mat.at<double>(0, 2),
        rot_mat.at<double>(1, 2),
        rot_mat.at<double>(2, 2));
    
    // 归一化方向向量
    float norm = cv::norm(armor_direction);
    armor_direction.x /= norm;
    armor_direction.y /= norm;
    armor_direction.z /= norm;
    
    // 3. 计算旋转中心
    return cv::Point3f(
        armor_center.x - rotation_radius * armor_direction.x,
        armor_center.y - rotation_radius * armor_direction.y,
        armor_center.z - rotation_radius * armor_direction.z);
}

cv::Point3f RotationCenterCalculator::calculateArmorCenterWorld(
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) {
    
    cv::Mat rvec, tvec;
    cv::solvePnP(getArmorObjectPoints(), image_points, camera_matrix, dist_coeffs, rvec, tvec);
    return cv::Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
}

std::vector<cv::Point3f> RotationCenterCalculator::getArmorObjectPoints() const {
    return {
        cv::Point3f(-armor_width_/2, -armor_height_/2, 0),
        cv::Point3f(armor_width_/2, -armor_height_/2, 0),
        cv::Point3f(armor_width_/2, armor_height_/2, 0),
        cv::Point3f(-armor_width_/2, armor_height_/2, 0)
    };
}