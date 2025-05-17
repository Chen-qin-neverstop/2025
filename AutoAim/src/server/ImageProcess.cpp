#include "ImageProcess.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

using namespace cv;
using namespace std;

// HSV阈值（可调全局变量）
int h_min = 46, h_max = 124;
int s_min = 92, s_max = 194;  // 195
int v_min = 217, v_max = 255;

// int h_min = 46,  h_max = 124;   // H 放宽（覆盖青蓝~蓝紫色）
// int s_min = 70,  s_max = 255;   // S 下限降低（允许浅蓝色）
// int v_min = 200,  v_max = 255;   // V 下限降低（适应暗光）

// 装甲板尺寸
const float ARMOR_WIDTH = 135.0f;
const float LIGHT_BAR_LENGTH = 55.0f;

// 通道相减阈值（可调全局变量）
int thres_max_color_red = 38;  // 红色通道相减阈值
int thres_max_color_blue = 36; // 蓝色通道相减阈值
int gray_threshold = 140;       // 灰度阈值  143

// 相机参数
const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
    2065.0580175762857, 0.0, 658.9098266395495,
    0.0, 2086.886458338243, 531.5333174739342,
    0.0, 0.0, 1.0);

const Mat DIST_COEFFS = (Mat_<double>(5, 1) << 
    -0.051836613762195866, 0.29341513924119095, 
    0.001501183796729562, 0.0009386915104617738, 0.0);

// Mat preprocessImage(const Mat &frame) {
//     Mat hsv, mask_red, mask_red1, mask_red2, result;
//     cvtColor(frame, hsv, COLOR_BGR2HSV);
//     inRange(hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), mask_red1);
//     inRange(hsv, Scalar(170, s_min, v_min), Scalar(180, s_max, v_max), mask_red2);
//     mask_red = mask_red1 | mask_red2;
//     Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
//     morphologyEx(mask_red, result, MORPH_CLOSE, kernel);
//     return result;
// }

// 颜色识别模式
enum ColorMode { RED, BLUE };
ColorMode current_color_mode = BLUE;  // 默认识别红色

Mat preprocessImage(const Mat &frame) {
    Mat binary_img, gray, thres_whole;
    vector<Mat> channels;
    
    // 分割BGR通道
    split(frame, channels);
    
    // 转换为灰度图并进行阈值处理
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    threshold(gray, thres_whole, gray_threshold, 255, THRESH_BINARY);
    
    // 根据颜色模式选择通道相减策略
    if (current_color_mode == RED) {
        // 红色识别：R通道 - B通道
        subtract(channels[2], channels[0], binary_img);
        threshold(binary_img, binary_img, thres_max_color_red, 255, THRESH_BINARY);
    } else {
        // 蓝色识别：B通道 - R通道
        subtract(channels[0], channels[2], binary_img);
        threshold(binary_img, binary_img, thres_max_color_blue, 255, THRESH_BINARY);
    }
    
    // 结合颜色阈值和整体灰度阈值
    binary_img = binary_img & thres_whole;
    
    // 形态学闭运算，填充小孔洞
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(binary_img, binary_img, MORPH_CLOSE, kernel);
    
    return binary_img;
}

vector<RotatedRect> findLightBars(const Mat &binary_img) {
    vector<vector<Point>> contours;
    vector<RotatedRect> light_bars;
    findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (const auto &cnt : contours) {
        float area = contourArea(cnt);
        if (area < 30.0f) continue;
        RotatedRect rect = minAreaRect(cnt);
        float length = max(rect.size.width, rect.size.height);
        float width = min(rect.size.width, rect.size.height);
        if (length/width > 4.0f && length > 20.0f) {
            light_bars.push_back(rect);
        }
    }
    return light_bars;
}

vector<pair<RotatedRect, RotatedRect>> matchArmorPairs(const vector<RotatedRect> &light_bars) {
    vector<pair<RotatedRect, RotatedRect>> armor_pairs;
    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const RotatedRect &rect1 = light_bars[i];
            const RotatedRect &rect2 = light_bars[j];
            // 角度差
            float angle_diff = abs(rect1.angle - rect2.angle);
            if (angle_diff > 10.0f && angle_diff < 170.0f) continue;  // 15  165
            // 距离
            float distance = norm(rect1.center - rect2.center);
            if (distance < ARMOR_WIDTH * 0.4 || distance > ARMOR_WIDTH * 1.6) continue;
            // 高度差
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
vector<Point2f> getArmorCorners(const pair<RotatedRect, RotatedRect> &light_pair) {
    const RotatedRect &left_light = light_pair.first;
    const RotatedRect &right_light = light_pair.second;
    // 获取左右灯条的角点(顺时针)
    Point2f left_pts[4], right_pts[4];
    left_light.points(left_pts);
    right_light.points(right_pts);
    // 对灯条顶点进行排序：确保顺序一致
    auto sortPoints = [](Point2f* pts) {
        //// 找到y值最大的两个点（灯条底部）
        sort(pts, pts + 4, [](const Point2f& a, const Point2f& b) {
            return a.y > b.y;
        });
        // 根据x坐标确定左右
        if (pts[0].x > pts[1].x) swap(pts[0], pts[1]);
        if (pts[2].x > pts[3].x) swap(pts[2], pts[3]);
    };
    sortPoints(left_pts);
    sortPoints(right_pts);
    vector<Point2f> armor_corners(4);
    // 左灯条的上顶点和下顶点
    Point2f left_top = (left_pts[2] + left_pts[3]) / 2;
    Point2f left_bottom = (left_pts[0] + left_pts[1]) / 2;
    // 右灯条的上顶点和下顶点
    Point2f right_top = (right_pts[2] + right_pts[3]) / 2;
    Point2f right_bottom = (right_pts[0] + right_pts[1]) / 2;
    // 装甲板角点
    armor_corners[0] = left_bottom;
    armor_corners[1] = right_bottom;
    armor_corners[2] = right_top;
    armor_corners[3] = left_top;
    // 返回装甲板角点
    return armor_corners;
}

void solveArmorPose(const vector<Point2f> &image_points, Mat &rvec, Mat &tvec) {
    vector<Point3f> object_points = {
        Point3f(-ARMOR_WIDTH/2, -LIGHT_BAR_LENGTH/2, 0),  
        Point3f(ARMOR_WIDTH/2, -LIGHT_BAR_LENGTH/2, 0),
        Point3f(ARMOR_WIDTH/2, LIGHT_BAR_LENGTH/2, 0),
        Point3f(-ARMOR_WIDTH/2, LIGHT_BAR_LENGTH/2, 0)
    };
    solvePnP(object_points, image_points, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec);
}

// 在图像上绘制装甲板和距离信息（精度提高到小数点后3位）
void drawDistanceInfo(Mat &image, float distance_mm,const vector<Point2f> &corners) {
    // 绘制装甲板
    for (int i = 0; i < 4; i++) {
        line(image, corners[i], corners[(i+1)%4], Scalar(0,255,0), 2);
    }
    
    // 计算装甲板中心点
    Point2f center1 = (corners[0] + corners[2]) / 2;
    Point2f center2 = (corners[1] + corners[3]) / 2;
    
    stringstream ss;
    ss << fixed << setprecision(3) << distance_mm/1000.0 << "m";
    
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int thickness = 2;
    int baseline = 0;
    
    // 计算文本大小
    Size text_size = getTextSize(ss.str(), font_face, font_scale, thickness, &baseline);
    
    // 绘制距离文本
    int padding = 5;
    Point text_tl(center1.x - text_size.width/2 - padding, center1.y - text_size.height - padding);
    Point text_br(center1.x + text_size.width/2 + padding, center1.y + padding);
    // 绘制距离文本
    putText(image, ss.str(), 
            Point(center1.x - text_size.width/2, center1.y - 5), 
            font_face, font_scale, Scalar(0, 255, 255), thickness);
}