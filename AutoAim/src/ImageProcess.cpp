#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>


using namespace cv;
using namespace std;

// 全局变量用于滑动条
// int h_min = 50, h_max = 111;
// int s_min = 89, s_max = 150;
// int v_min = 124, v_max = 255;
int h_min = 46, h_max = 124;
int s_min = 92, s_max = 194;  // 195
int v_min = 217, v_max = 255;
const string window_name = "HSV Adjustment";

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

// // 滑动条回调函数
// void onTrackbar(int, void*) {
//     // 空实现，仅用于触发更新
// }

Mat preprocessImage(const Mat &frame) {
    Mat hsv, mask_red, mask_red1, mask_red2, result;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    // 使用滑动条调整的阈值
    inRange(hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), mask_red1);
    inRange(hsv, Scalar(170, s_min, v_min), Scalar(180, s_max, v_max), mask_red2);
    mask_red = mask_red1 | mask_red2;

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask_red, result, MORPH_CLOSE, kernel);
    return result;
}

// 查找灯条
vector<RotatedRect> findLightBars(const Mat &binary_img) {
    vector<vector<Point>> contours;
    vector<RotatedRect> light_bars;
    findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);   // 灯条找轮廓

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

// 匹配装甲板对
vector<pair<RotatedRect, RotatedRect>> matchArmorPairs(const vector<RotatedRect> &light_bars) {
    vector<pair<RotatedRect, RotatedRect>> armor_pairs;

    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const RotatedRect &rect1 = light_bars[i];
            const RotatedRect &rect2 = light_bars[j];

            // 角度差约束
            float angle_diff = abs(rect1.angle - rect2.angle);
            if (angle_diff > 15.0f && angle_diff < 165.0f) continue;

            // 距离约束
            float distance = norm(rect1.center - rect2.center);
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

// 精确获取装甲板角点
vector<Point2f> getArmorCorners(const pair<RotatedRect, RotatedRect> &light_pair) {
    const RotatedRect &left_light = light_pair.first;
    const RotatedRect &right_light = light_pair.second;
    
    // 获取灯条的四个顶点（按顺时针顺序）
    Point2f left_pts[4], right_pts[4];
    left_light.points(left_pts);
    right_light.points(right_pts);
    
    // 对灯条顶点进行排序：确保顺序一致
    auto sortPoints = [](Point2f* pts) {
        // 找到y值最大的两个点（灯条底部）
        sort(pts, pts + 4, [](const Point2f& a, const Point2f& b) {
            return a.y > b.y;
        });
        // 根据x坐标确定左右
        if (pts[0].x > pts[1].x) swap(pts[0], pts[1]);
        if (pts[2].x > pts[3].x) swap(pts[2], pts[3]);
    };
    
    sortPoints(left_pts);
    sortPoints(right_pts);
    
    // 装甲板角点计算
    vector<Point2f> armor_corners(4);
    
    // 左灯条的上顶点和下顶点
    Point2f left_top = (left_pts[2] + left_pts[3]) / 2;
    Point2f left_bottom = (left_pts[0] + left_pts[1]) / 2;
    
    // 右灯条的上顶点和下顶点
    Point2f right_top = (right_pts[2] + right_pts[3]) / 2;
    Point2f right_bottom = (right_pts[0] + right_pts[1]) / 2;
    
    // 计算装甲板角点
    armor_corners[0] = left_bottom;   // 左下
    armor_corners[1] = right_bottom;  // 右下
    armor_corners[2] = right_top;     // 右上
    armor_corners[3] = left_top;      // 左上
    
    // 应用几何约束优化角点位置
    const float armor_ratio = ARMOR_WIDTH / LIGHT_BAR_LENGTH;
    float current_ratio = norm(right_bottom - left_bottom) / 
                         ((norm(left_top - left_bottom) + norm(right_top - right_bottom)) * 2);
    
    return armor_corners;
}

// PnP解算 
void solveArmorPose(const vector<Point2f> &image_points, Mat &rvec, Mat &tvec) {
    vector<Point3f> object_points = {
        Point3f(-ARMOR_WIDTH/2, -LIGHT_BAR_LENGTH/2, 0),
        Point3f(ARMOR_WIDTH/2, -LIGHT_BAR_LENGTH/2, 0),
        Point3f(ARMOR_WIDTH/2, LIGHT_BAR_LENGTH/2, 0),
        Point3f(-ARMOR_WIDTH/2, LIGHT_BAR_LENGTH/2, 0)
    };
    solvePnP(object_points, image_points, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec);
}

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
    
    // // 创建滑动条
    // createTrackbar("H Min", window_name, &h_min, 180, onTrackbar);
    // createTrackbar("H Max", window_name, &h_max, 180, onTrackbar);
    // createTrackbar("S Min", window_name, &s_min, 255, onTrackbar);
    // createTrackbar("S Max", window_name, &s_max, 255, onTrackbar);
    // createTrackbar("V Min", window_name, &v_min, 255, onTrackbar);
    // createTrackbar("V Max", window_name, &v_max, 255, onTrackbar);

    Mat frame = imread("/home/chen/Project/Vscode/Code/RM/RM-task/Opencv/Armor_detectionTRUE/detection4/23.jpg");
    if (frame.empty()) {
        cerr << "Error: Could not load image!" << endl;
        return -1;
    }

    while (true) {
        Mat undistorted_frame;
        undistort(frame, undistorted_frame, CAMERA_MATRIX, DIST_COEFFS);

        // 实时处理
        Mat binary_img = preprocessImage(undistorted_frame);
        
        // 显示处理结果
        imshow("Binary Preview", binary_img);
        
        // 完整处理流程
        vector<RotatedRect> light_bars = findLightBars(binary_img);
        vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);
        
        Mat result_img = undistorted_frame.clone();
        for (const auto &pair : armor_pairs) {
            vector<Point2f> corners = getArmorCorners(pair);
            Mat rvec, tvec;
            solveArmorPose(corners, rvec, tvec);
            
            // 计算装甲板中心点和距离
            Point2f armor_center = (corners[0] + corners[2]) / 2;
            float distance_mm = norm(tvec);
            
            // 绘制装甲板
            for (int i = 0; i < 4; i++) {
                line(result_img, corners[i], corners[(i+1)%4], Scalar(0,255,0), 2);
            }
            
            // 绘制坐标系
            vector<Point3f> axis = {Point3f(0,0,0), Point3f(50,0,0), Point3f(0,50,0), Point3f(0,0,50)};
            vector<Point2f> projected_axis;
            projectPoints(axis, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS, projected_axis);
            arrowedLine(result_img, projected_axis[0], projected_axis[1], Scalar(0,0,255), 2); // X
            arrowedLine(result_img, projected_axis[0], projected_axis[2], Scalar(0,255,0), 2); // Y
            arrowedLine(result_img, projected_axis[0], projected_axis[3], Scalar(255,0,0), 2); // Z
            
            // 输出结果
            cout << "Rotation Vector:\n" << rvec << endl;
            cout << "Translation Vector (mm):\n" << tvec << endl;
            cout << "Distance: " << distance_mm << " mm" << endl;
            
            // 绘制距离信息
            drawDistanceInfo(result_img, armor_center, distance_mm);
        }
        imshow("Detection Result", result_img);

        // 退出条件
        int key = waitKey(30);
        if (key == 27) break; // ESC退出
        // if (key == 's') { // 保存当前阈值
        //     cout << "Current HSV Threshold:\n";
        //     cout << "H: [" << h_min << ", " << h_max << "]\n";
        //     cout << "S: [" << s_min << ", " << s_max << "]\n";
        //     cout << "V: [" << v_min << ", " << v_max << "]" << endl;
        // }
    }

    destroyAllWindows();
    return 0;
}
