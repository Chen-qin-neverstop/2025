#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;

// 全局变量用于存储HSV值
struct HSVParams {
    int h_min = 49;
    int h_max = 116;
    int s_min = 61;
    int s_max = 198;
    int v_min = 111;
    int v_max = 255;
} hsv_params;

const string window_name = "HSV Adjustment";

// 装甲板尺寸
const float ARMOR_WIDTH = 150.0f;
const float LIGHT_BAR_LENGTH = 60.0f;

// 相机参数
const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
    2065.0580175762857, 0.0, 658.9098266395495,
    0.0, 2086.886458338243, 531.5333174739342,
    0.0, 0.0, 1.0);

const Mat DIST_COEFFS = (Mat_<double>(5, 1) << 
    -0.051836613762195866, 0.29341513924119095, 
    0.001501183796729562, 0.0009386915104617738, 0.0);

// 滑动条回调函数
void onTrackbar(int, void*) {
    // 空实现，仅用于触发更新
}

Mat preprocessImage(const Mat &frame) {
    Mat hsv, mask_red, mask_red1, mask_red2, result;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    // 使用调整后的HSV阈值
    inRange(hsv, Scalar(hsv_params.h_min, hsv_params.s_min, hsv_params.v_min), Scalar(hsv_params.h_max, hsv_params.s_max, hsv_params.v_max), mask_red1);
    inRange(hsv, Scalar(170, hsv_params.s_min, hsv_params.v_min), Scalar(180, hsv_params.s_max, hsv_params.v_max), mask_red2);
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
        if (area < 40.0f) continue;

        RotatedRect rect = minAreaRect(cnt);
        float length = max(rect.size.width, rect.size.height);
        float width = min(rect.size.width, rect.size.height);

        if (length/width > 3.5f && length > 25.0f) {
            light_bars.push_back(rect);
        }
    }
    return light_bars;
}

vector<pair<Point2f, Point2f>> matchArmorPairs(const vector<RotatedRect> &light_bars) {
    vector<pair<Point2f, Point2f>> armor_pairs;

    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const RotatedRect &rect1 = light_bars[i];
            const RotatedRect &rect2 = light_bars[j];

            float angle_diff = abs(rect1.angle - rect2.angle);
            if (angle_diff > 20.0f && angle_diff < 160.0f) continue;

            float distance = norm(rect1.center - rect2.center);
            if (distance < ARMOR_WIDTH * 0.5 || distance > ARMOR_WIDTH * 1.5) continue;

            if (abs(rect1.center.y - rect2.center.y) > LIGHT_BAR_LENGTH/2.5f) continue;

            armor_pairs.emplace_back(rect1.center, rect2.center);
        }
    }
    return armor_pairs;
}

vector<Point2f> getArmorCorners(const pair<Point2f, Point2f> &pair) {
    Point2f p1 = pair.first;
    Point2f p2 = pair.second;

    Point2f dir = p2 - p1;
    Point2f perpendicular(-dir.y, dir.x);
    perpendicular = perpendicular / norm(perpendicular) * (LIGHT_BAR_LENGTH/2.0f);

    return {
        p1 + perpendicular,
        p2 + perpendicular,
        p2 - perpendicular,
        p1 - perpendicular
    };
}
// 在图像上绘制距离信息
// 在图像上绘制距离信息（精度提高到小数点后3位）
void drawDistanceInfo(Mat &image, const Point2f &center, float distance_mm) {
    stringstream ss;
    ss << fixed << setprecision(3) << distance_mm/1000.0 << "m";
    
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int thickness = 2;
    int baseline = 0;
    
    // 计算文本大小
    Size text_size = getTextSize(ss.str(), font_face, font_scale, thickness, &baseline);
    
    // 绘制背景矩形（带圆角效果）
    Rect bg_rect(center.x - text_size.width/2 - 5, 
                center.y - text_size.height - 10,
                text_size.width + 10, 
                text_size.height + 10);
    rectangle(image, bg_rect, Scalar(0, 0, 0), -1);
    
    // 绘制白色边框
    rectangle(image, bg_rect, Scalar(255, 255, 255), 1);
    
    // 绘制距离文本
    putText(image, ss.str(), 
            Point(center.x - text_size.width/2, center.y - 5), 
            font_face, font_scale, Scalar(0, 255, 255), thickness);
}

int main() {
    // 创建调整窗口
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 600, 300);
    
    // 创建滑动条
    createTrackbar("H Min", window_name, nullptr, 180, onTrackbar);
    setTrackbarPos("H Min", window_name, hsv_params.h_min);
    createTrackbar("H Max", window_name, nullptr, 180, onTrackbar);
    setTrackbarPos("H Max", window_name, hsv_params.h_max);
    createTrackbar("S Min", window_name, nullptr, 255, onTrackbar);
    setTrackbarPos("S Min", window_name, hsv_params.s_min);
    createTrackbar("S Max", window_name, nullptr, 255, onTrackbar);
    setTrackbarPos("S Max", window_name, hsv_params.s_max);
    createTrackbar("V Min", window_name, nullptr, 255, onTrackbar);
    setTrackbarPos("V Min", window_name, hsv_params.v_min);
    createTrackbar("V Max", window_name, nullptr, 255, onTrackbar);
    setTrackbarPos("V Max", window_name, hsv_params.v_max);

    Mat frame = imread("9.jpg");
    if (frame.empty()) {
        cerr << "Error: Could not load image!" << endl;
        return -1;
    }

    while (true) {
        Mat undistorted_frame;
        undistort(frame, undistorted_frame, CAMERA_MATRIX, DIST_COEFFS);

        // 获取当前滑动条位置
        hsv_params.h_min = getTrackbarPos("H Min", window_name);
        hsv_params.h_max = getTrackbarPos("H Max", window_name);
        hsv_params.s_min = getTrackbarPos("S Min", window_name);
        hsv_params.s_max = getTrackbarPos("S Max", window_name);
        hsv_params.v_min = getTrackbarPos("V Min", window_name);
        hsv_params.v_max = getTrackbarPos("V Max", window_name);

        // 实时处理
        Mat binary_img = preprocessImage(undistorted_frame);
        vector<RotatedRect> light_bars = findLightBars(binary_img);
        vector<pair<Point2f, Point2f>> armor_pairs = matchArmorPairs(light_bars);
        
        Mat result_img = undistorted_frame.clone();
        for (const auto &pair : armor_pairs) {
            vector<Point2f> corners = getArmorCorners(pair);
            Mat rvec, tvec;
            solveArmorPose(corners, rvec, tvec);
            
            // 计算装甲板中心点
            Point2f armor_center = (pair.first + pair.second) / 2.0f;
            
            // 绘制装甲板
            for (int i = 0; i < 4; i++) {
                line(result_img, corners[i], corners[(i+1)%4], Scalar(0,255,0), 2);
            }
            
            // 绘制距离信息（精度0.001米）
            float distance_mm = norm(tvec);
            drawDistanceInfo(result_img, armor_center, distance_mm);
            
            // 绘制坐标系
            vector<Point3f> axis = {Point3f(0,0,0), Point3f(50,0,0), Point3f(0,50,0), Point3f(0,0,50)};
            vector<Point2f> projected_axis;
            projectPoints(axis, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS, projected_axis);
            arrowedLine(result_img, projected_axis[0], projected_axis[1], Scalar(0,0,255), 2); // X
            arrowedLine(result_img, projected_axis[0], projected_axis[2], Scalar(0,255,0), 2); // Y
            arrowedLine(result_img, projected_axis[0], projected_axis[3], Scalar(255,0,0), 2); // Z
            
            // 控制台输出也保持3位小数
            cout << fixed << setprecision(3);
            cout << "Distance: " << distance_mm/1000.0 << "m" << endl;
        }
        
        imshow("Binary Preview", binary_img);
        imshow("Detection Result", result_img);

        // 退出条件
        int key = waitKey(30);
        if (key == 27) break;
        if (key == 's') {
            cout << "Current HSV Threshold:\n";
            cout << "H: [" << hsv_params.h_min << ", " << hsv_params.h_max << "]\n";
            cout << "S: [" << hsv_params.s_min << ", " << hsv_params.s_max << "]\n";
            cout << "V: [" << hsv_params.v_min << ", " << hsv_params.v_max << "]" << endl;
        }
    }

    destroyAllWindows();
    return 0;
}
