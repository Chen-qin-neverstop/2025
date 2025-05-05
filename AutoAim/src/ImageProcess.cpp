#include "ImageProcess.h"
#include <iostream>

// 相机参数
const cv::Mat CAMERA_MATRIX = (cv::Mat_<double>(3, 3) <<
    2065.0580175762857, 0.0, 658.9098266395495,
    0.0, 2086.886458338243, 531.5333174739342,
    0.0, 0.0, 1.0);

const cv::Mat DIST_COEFFS = (cv::Mat_<double>(5, 1) << 
    -0.051836613762195866, 0.29341513924119095, 
    0.001501183796729562, 0.0009386915104617738, 0.0);

// 装甲板尺寸 (单位:米)
const float ARMOR_WIDTH = 0.135f;
const float LIGHT_BAR_LENGTH = 0.055f;

// 预处理图像
cv::Mat preprocessImage(const cv::Mat &frame) {
    cv::Mat hsv, mask_red, mask_red1, mask_red2, result;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // 红色阈值范围
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask_red2);
    mask_red = mask_red1 | mask_red2;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_red, result, cv::MORPH_CLOSE, kernel);
    return result;
}

// 查找灯条
std::vector<cv::RotatedRect> findLightBars(const cv::Mat &binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::RotatedRect> light_bars;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto &cnt : contours) {
        float area = cv::contourArea(cnt);
        if (area < 30.0f) continue;

        cv::RotatedRect rect = cv::minAreaRect(cnt);
        float length = std::max(rect.size.width, rect.size.height);
        float width = std::min(rect.size.width, rect.size.height);

        if (length/width > 4.0f && length > 20.0f) {
            light_bars.push_back(rect);
        }
    }
    return light_bars;
}

// 匹配装甲板对
std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> matchArmorPairs(const std::vector<cv::RotatedRect> &light_bars) {
    std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armor_pairs;

    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const cv::RotatedRect &rect1 = light_bars[i];
            const cv::RotatedRect &rect2 = light_bars[j];

            // 角度差约束
            float angle_diff = abs(rect1.angle - rect2.angle);
            if (angle_diff > 15.0f && angle_diff < 165.0f) continue;

            // 距离约束
            float distance = cv::norm(rect1.center - rect2.center);
            if (distance < ARMOR_WIDTH * 0.6 || distance > ARMOR_WIDTH * 1.4) continue;

            // 高度差约束
            if (abs(rect1.center.y - rect2.center.y) > LIGHT_BAR_LENGTH/3.0f) continue;

            // 确保左右顺序
            if (rect1.center.x < rect2.center.x) {
                armor_pairs.emplace_back(rect1, rect2);
            } else {
                armor_pairs.emplace_back(rect2, rect1);
            }
        }
    }
    return armor_pairs;
}

// 获取装甲板角点
std::vector<cv::Point2f> getArmorCorners(const std::pair<cv::RotatedRect, cv::RotatedRect> &light_pair) {
    const cv::RotatedRect &left_light = light_pair.first;
    const cv::RotatedRect &right_light = light_pair.second;
    
    cv::Point2f left_pts[4], right_pts[4];
    left_light.points(left_pts);
    right_light.points(right_pts);
    
    // 对灯条顶点进行排序
    auto sortPoints = [](cv::Point2f* pts) {
        std::sort(pts, pts + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.y > b.y;
        });
        if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
        if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);
    };
    
    sortPoints(left_pts);
    sortPoints(right_pts);
    
    // 计算装甲板角点
    std::vector<cv::Point2f> armor_corners(4);
    
    // 左灯条的上顶点和下顶点
    cv::Point2f left_top = (left_pts[2] + left_pts[3]) / 2;
    cv::Point2f left_bottom = (left_pts[0] + left_pts[1]) / 2;
    
    // 右灯条的上顶点和下顶点
    cv::Point2f right_top = (right_pts[2] + right_pts[3]) / 2;
    cv::Point2f right_bottom = (right_pts[0] + right_pts[1]) / 2;
    
    // 装甲板角点
    armor_corners[0] = left_bottom;    // 左下
    armor_corners[1] = right_bottom;   // 右下
    armor_corners[2] = right_top;      // 右上
    armor_corners[3] = left_top;       // 左上
    
    return armor_corners;
}

// PnP解算位姿
void solveArmorPose(const std::vector<cv::Point2f> &image_points, cv::Mat &rvec, cv::Mat &tvec) {
    std::vector<cv::Point3f> object_points = {
        cv::Point3f(-ARMOR_WIDTH/2, -LIGHT_BAR_LENGTH/2, 0),
        cv::Point3f(ARMOR_WIDTH/2, -LIGHT_BAR_LENGTH/2, 0),
        cv::Point3f(ARMOR_WIDTH/2, LIGHT_BAR_LENGTH/2, 0),
        cv::Point3f(-ARMOR_WIDTH/2, LIGHT_BAR_LENGTH/2, 0)
    };
    cv::solvePnP(object_points, image_points, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec);
}

// 绘制距离信息
void drawDistanceInfo(cv::Mat &image, const cv::Point2f &center, float distance_mm) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << distance_mm/1000.0 << "m";
    
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int thickness = 2;
    int baseline = 0;
    
    cv::Size text_size = cv::getTextSize(ss.str(), font_face, font_scale, thickness, &baseline);
    
    cv::Rect bg_rect(center.x - text_size.width/2 - 5, 
                    center.y - text_size.height - 10,
                    text_size.width + 10, 
                    text_size.height + 10);
    cv::rectangle(image, bg_rect, cv::Scalar(0, 0, 0), -1);
    cv::rectangle(image, bg_rect, cv::Scalar(255, 255, 255), 1);
    
    cv::putText(image, ss.str(), 
                cv::Point(center.x - text_size.width/2, center.y - 5), 
                font_face, font_scale, cv::Scalar(0, 255, 255), thickness);
}