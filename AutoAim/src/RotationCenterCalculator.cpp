#include "RotationCenterCalculator.h"
#include <iostream>

using namespace cv;
using namespace std;

Point3f RotationCenterCalculator::calculateRotationCenter(
    const vector<Point2f>& image_points,
    const Mat& camera_matrix,
    const Mat& dist_coeffs,
    float rotation_radius) {
    
    // 1. 计算装甲板中心在世界坐标系中的位置
    Point3f armor_center = calculateArmorCenterWorld(image_points, camera_matrix, dist_coeffs);
    
    // 2. 计算装甲板朝向向量
    Mat rvec, tvec;
    solvePnP(getArmorObjectPoints(), image_points, camera_matrix, dist_coeffs, rvec, tvec);
    
    Mat rot_mat;
    Rodrigues(rvec, rot_mat);
    
    // 装甲板正方向向量 (Z轴方向)
    Point3f armor_direction(
        rot_mat.at<double>(0, 2),
        rot_mat.at<double>(1, 2),
        rot_mat.at<double>(2, 2));
    
    // 归一化方向向量
    float norm = sqrt(armor_direction.x*armor_direction.x + 
                     armor_direction.y*armor_direction.y + 
                     armor_direction.z*armor_direction.z);
    armor_direction.x /= norm;
    armor_direction.y /= norm;
    armor_direction.z /= norm;
    
    // 3. 计算旋转中心
    // 旋转中心 = 装甲板中心 + 旋转半径 * 装甲板朝向的反方向
    Point3f rotation_center(
        armor_center.x - rotation_radius * armor_direction.x,
        armor_center.y - rotation_radius * armor_direction.y,
        armor_center.z - rotation_radius * armor_direction.z);
    
    return rotation_center;
}

Point3f RotationCenterCalculator::calculateArmorCenterWorld(
    const vector<Point2f>& image_points,
    const Mat& camera_matrix,
    const Mat& dist_coeffs) {
    
    Mat rvec, tvec;
    solvePnP(getArmorObjectPoints(), image_points, camera_matrix, dist_coeffs, rvec, tvec);
    
    // 装甲板中心在世界坐标系中的位置就是平移向量tvec
    return Point3f(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2));
}

vector<Point3f> RotationCenterCalculator::getArmorObjectPoints() const {
    return {
        Point3f(-armor_width_/2, -armor_height_/2, 0),  // 左下
        Point3f(armor_width_/2, -armor_height_/2, 0),   // 右下
        Point3f(armor_width_/2, armor_height_/2, 0),    // 右上
        Point3f(-armor_width_/2, armor_height_/2, 0)    // 左上
    };
}