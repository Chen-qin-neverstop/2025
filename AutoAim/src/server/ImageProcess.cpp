#include "ImageProcess.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

using namespace cv;
using namespace std;

// 全局变量用于滑动条
int h_min = 46, h_max = 124;
int s_min = 92, s_max = 194;
int v_min = 217, v_max = 255;
// const string window_name = "HSV Adjustment"; // 删除滑动条窗口名

// 装甲板尺寸
const float ARMOR_WIDTH = 135.0f;
const float LIGHT_BAR_LENGTH = 55.0f;

// 相机参数
const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
    2065.0580175762857, 0.0, 658.9098266395495,
    0.0, 2086.886458338243, 531.5333174739342,
    0.0, 0.0, 1.0);

const Mat DIST_COEFFS = (Mat_<double>(5, 1) << 
    -0.051836613762195866, 0.29341513924119095, 
    0.001501183796729562, 0.0009386915104617738, 0.0);

// void onTrackbar(int, void*) {} // 删除滑动条回调

Mat preprocessImage(const Mat &frame) {
    Mat hsv, mask_red, mask_red1, mask_red2, result;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), mask_red1);
    inRange(hsv, Scalar(170, s_min, v_min), Scalar(180, s_max, v_max), mask_red2);
    mask_red = mask_red1 | mask_red2;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask_red, result, MORPH_CLOSE, kernel);
    return result;
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
            float angle_diff = abs(rect1.angle - rect2.angle);
            if (angle_diff > 15.0f && angle_diff < 165.0f) continue;
            float distance = norm(rect1.center - rect2.center);
            if (distance < ARMOR_WIDTH * 0.6 || distance > ARMOR_WIDTH * 1.4) continue;
            if (abs(rect1.center.y - rect2.center.y) > LIGHT_BAR_LENGTH/3.0f) continue;
            if (rect1.center.x < rect2.center.x) {
                armor_pairs.emplace_back(rect1, rect2);
            } else {
                armor_pairs.emplace_back(rect2, rect1);
            }
        }
    }
    return armor_pairs;
}

vector<Point2f> getArmorCorners(const pair<RotatedRect, RotatedRect> &light_pair) {
    const RotatedRect &left_light = light_pair.first;
    const RotatedRect &right_light = light_pair.second;
    Point2f left_pts[4], right_pts[4];
    left_light.points(left_pts);
    right_light.points(right_pts);
    auto sortPoints = [](Point2f* pts) {
        sort(pts, pts + 4, [](const Point2f& a, const Point2f& b) {
            return a.y > b.y;
        });
        if (pts[0].x > pts[1].x) swap(pts[0], pts[1]);
        if (pts[2].x > pts[3].x) swap(pts[2], pts[3]);
    };
    sortPoints(left_pts);
    sortPoints(right_pts);
    vector<Point2f> armor_corners(4);
    Point2f left_top = (left_pts[2] + left_pts[3]) / 2;
    Point2f left_bottom = (left_pts[0] + left_pts[1]) / 2;
    Point2f right_top = (right_pts[2] + right_pts[3]) / 2;
    Point2f right_bottom = (right_pts[0] + right_pts[1]) / 2;
    armor_corners[0] = left_bottom;
    armor_corners[1] = right_bottom;
    armor_corners[2] = right_top;
    armor_corners[3] = left_top;
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

void drawDistanceInfo(Mat &image, const Point2f &center, float distance_mm) {
    stringstream ss;
    ss << fixed << setprecision(3) << distance_mm/1000.0 << "m";
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int thickness = 2;
    int baseline = 0;
    Size text_size = getTextSize(ss.str(), font_face, font_scale, thickness, &baseline);
    Rect bg_rect(center.x - text_size.width/2 - 5, 
                center.y - text_size.height - 10,
                text_size.width + 10, 
                text_size.height + 10);
    rectangle(image, bg_rect, Scalar(0, 0, 0), -1);
    rectangle(image, bg_rect, Scalar(255, 255, 255), 1);
    putText(image, ss.str(), 
            Point(center.x - text_size.width/2, center.y - 5), 
            font_face, font_scale, Scalar(0, 255, 255), thickness);
}