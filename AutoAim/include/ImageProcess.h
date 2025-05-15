#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <opencv2/opencv.hpp>
#include <vector>

// 相机参数
extern const cv::Mat CAMERA_MATRIX;
extern const cv::Mat DIST_COEFFS;

// 装甲板尺寸
extern const float ARMOR_WIDTH;
extern const float LIGHT_BAR_LENGTH;

cv::Mat preprocessImage(const cv::Mat &frame);
std::vector<cv::RotatedRect> findLightBars(const cv::Mat &binary_img);
std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> matchArmorPairs(const std::vector<cv::RotatedRect> &light_bars);
std::vector<cv::Point2f> getArmorCorners(const std::pair<cv::RotatedRect, cv::RotatedRect> &light_pair);
void solveArmorPose(const std::vector<cv::Point2f> &image_points, cv::Mat &rvec, cv::Mat &tvec);
void drawDistanceInfo(cv::Mat &image, float distance_mm,const std::vector<cv::Point2f>& corners);

#endif