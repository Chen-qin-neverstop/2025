#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;

// 全局变量用于滑动条（调整为BGR相关参数）
int blue_thresh = 50;  // 蓝色通道阈值
int blue_boost = 30;   // 蓝色增强系数
const string window_name = "BGR Adjustment";

// 装甲板尺寸和相机参数（保持不变）
const float ARMOR_WIDTH = 135.0f;
const float LIGHT_BAR_LENGTH = 55.0f;
const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<
    2065.0580175762857, 0.0, 658.9098266395495,
    0.0, 2086.886458338243, 531.5333174739342,
    0.0, 0.0, 1.0);
const Mat DIST_COEFFS = (Mat_<double>(5, 1) << 
    -0.051836613762195866, 0.29341513924119095, 
    0.001501183796729562, 0.0009386915104617738, 0.0;

// 滑动条回调函数
void onTrackbar(int, void*) {}

// 新版预处理函数：BGR通道分离提取蓝色
Mat preprocessImage(const Mat &frame) {
    vector<Mat> channels;
    split(frame, channels);  // 分离BGR通道
    
    // 蓝色通道强化：B - (G + R)/2 + boost
    Mat blue_enhanced;
    subtract(channels[0], (channels[1] + channels[2]) / 2, blue_enhanced);
    add(blue_enhanced, blue_boost, blue_enhanced);
    
    // 二值化
    Mat binary;
    threshold(blue_enhanced, binary, blue_thresh, 255, THRESH_BINARY);
    
    // 形态学处理
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(binary, binary, MORPH_CLOSE, kernel);
    
    return binary;
}

// ...（保持原有的findLightBars、fitLightBarPolygon、matchArmorPairs、
//     getArmorCorners、solveArmorPose、drawDistanceInfo函数不变）...
vector<RotatedRect> findLightBars(const Mat &binary_img) {
    vector<vector<Point>> contours; // 用于存储轮廓
    vector<RotatedRect> light_bars; // 用于存储检测到的灯条

    // 查找二值图像中的轮廓
    findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 遍历所有轮廓
    for (const auto &cnt : contours) {
        float area = contourArea(cnt); // 计算轮廓面积
        if (area < 30.0f) continue; // 忽略面积过小的轮廓

        // 获取轮廓的最小外接旋转矩形
        RotatedRect rect = minAreaRect(cnt);
        float length = max(rect.size.width, rect.size.height); // 获取矩形的长边
        float width = min(rect.size.width, rect.size.height);  // 获取矩形的短边

        // 判断矩形是否符合灯条的形状特征
        if (length / width > 4.0f && length > 20.0f) {
            light_bars.push_back(rect); // 将符合条件的矩形添加到灯条列表中
        }
    }
    return light_bars; // 返回检测到的灯条
}

// 精确拟合灯带多边形
vector<Point2f> fitLightBarPolygon(const RotatedRect &light_bar) {
    vector<Point2f> polygon;
    
    // 获取旋转矩形的四个顶点
    Point2f vertices[4];
    light_bar.points(vertices);
    
    // 对顶点进行排序（从上到下，从左到右）
    sort(vertices, vertices + 4, [](const Point2f &a, const Point2f &b) {
        return a.y < b.y || (a.y == b.y && a.x < b.x);
    });
    
    // 调整顺序为顺时针
    polygon.push_back(vertices[0]);
    polygon.push_back(vertices[1]);
    polygon.push_back(vertices[3]);
    polygon.push_back(vertices[2]);
    
    return polygon;
}

vector<pair<RotatedRect, RotatedRect>> matchArmorPairs(const vector<RotatedRect> &light_bars) {
    vector<pair<RotatedRect, RotatedRect>> armor_pairs; // 用于存储匹配的装甲板对

    // 遍历所有可能的光条组合
    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const RotatedRect &rect1 = light_bars[i]; // 第一个光条
            const RotatedRect &rect2 = light_bars[j]; // 第二个光条

            // 计算两个光条的角度差
            float angle_diff = abs(rect1.angle - rect2.angle);
            // 如果角度差不符合条件（大于15度且小于165度），跳过当前组合
            if (angle_diff > 15.0f && angle_diff < 165.0f) continue;

            // 计算两个光条中心点之间的距离
            float distance = norm(rect1.center - rect2.center);
            // 如果距离不符合条件（小于装甲板宽度的0.6倍或大于1.4倍），跳过当前组合
            if (distance < ARMOR_WIDTH * 0.6 || distance > ARMOR_WIDTH * 1.4) continue;

            // 检查两个光条在垂直方向上的高度差
            if (abs(rect1.center.y - rect2.center.y) > LIGHT_BAR_LENGTH / 3.0f) continue;

            // 确保光条按左右顺序排列
            if (rect1.center.x < rect2.center.x) {
                armor_pairs.emplace_back(rect1, rect2); // 左光条在前
            } else {
                armor_pairs.emplace_back(rect2, rect1); // 右光条在前
            }
        }
    }
    return armor_pairs; // 返回所有符合条件的装甲板对
}
// 精确获取装甲板角点 getArmorCorners 
vector<Point2f> getArmorCorners(const pair<RotatedRect, RotatedRect> &light_pair) {
    const RotatedRect &left_light = light_pair.first;
    const RotatedRect &right_light = light_pair.second;
    
    vector<Point2f> left_poly = fitLightBarPolygon(left_light);
    vector<Point2f> right_poly = fitLightBarPolygon(right_light);
    
    // 装甲板角点
    vector<Point2f> armor_corners(4);
    armor_corners[0] = (left_poly[2] + left_poly[3]) / 2;  // 左下
    armor_corners[1] = (right_poly[2] + right_poly[3]) / 2; // 右下
    armor_corners[2] = (right_poly[0] + right_poly[1]) / 2; // 右上
    armor_corners[3] = (left_poly[0] + left_poly[1]) / 2;   // 左上
    
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
// 绘制距离信息
void drawDistanceInfo(Mat &image, const Point2f &center, float distance_mm) {
    stringstream ss;
    ss << fixed << setprecision(3) << distance_mm/1000.0 << "m";
    
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int thickness = 2;
    
    Size text_size = getTextSize(ss.str(), font_face, font_scale, thickness, 0);
    
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


int main() {
    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 600, 300);
    
    // 创建BGR调整滑动条
    createTrackbar("Blue Threshold", window_name, &blue_thresh, 255, onTrackbar);
    createTrackbar("Blue Boost", window_name, &blue_boost, 100, onTrackbar);

    Mat frame = imread("15.jpg");
    if (frame.empty()) {
        cerr << "Error: Could not load image!" << endl;
        return -1;
    }

    while (true) {
        Mat undistorted_frame;
        undistort(frame, undistorted_frame, CAMERA_MATRIX, DIST_COEFFS);

        // 使用BGR分离预处理
        Mat binary_img = preprocessImage(undistorted_frame);
        imshow("Binary Preview", binary_img);
        
        // 后续处理保持不变
        vector<RotatedRect> light_bars = findLightBars(binary_img);
        vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);
        
        Mat result_img = undistorted_frame.clone();
        
        // 绘制所有灯带
        for (const auto &light : light_bars) {
            vector<Point2f> light_poly = fitLightBarPolygon(light);
            for (size_t i = 0; i < light_poly.size(); i++) {
                line(result_img, light_poly[i], light_poly[(i+1)%4], Scalar(255, 0, 0), 2);
            }
        }
        
        for (const auto &pair : armor_pairs) {
            // 绘制匹配的灯带对和装甲板
            vector<Point2f> left_poly = fitLightBarPolygon(pair.first);
            vector<Point2f> right_poly = fitLightBarPolygon(pair.second);
            
            for (size_t i = 0; i < 4; i++) {
                line(result_img, left_poly[i], left_poly[(i+1)%4], Scalar(0, 165, 255), 3);
                line(result_img, right_poly[i], right_poly[(i+1)%4], Scalar(255, 0, 255), 3);
            }
            
            vector<Point2f> corners = getArmorCorners(pair);
            Mat rvec, tvec;
            solveArmorPose(corners, rvec, tvec);
            
            Point2f armor_center = (corners[0] + corners[2]) / 2;
            float distance_mm = norm(tvec);
            
            for (int i = 0; i < 4; i++) {
                line(result_img, corners[i], corners[(i+1)%4], Scalar(0, 255, 0), 2);
            }
            
            vector<Point3f> axis = {Point3f(0,0,0), Point3f(50,0,0), Point3f(0,50,0), Point3f(0,0,50)};
            vector<Point2f> projected_axis;
            projectPoints(axis, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS, projected_axis);
            arrowedLine(result_img, projected_axis[0], projected_axis[1], Scalar(0,0,255), 2);
            arrowedLine(result_img, projected_axis[0], projected_axis[2], Scalar(0,255,0), 2);
            arrowedLine(result_img, projected_axis[0], projected_axis[3], Scalar(255,0,0), 2);
            
            drawDistanceInfo(result_img, armor_center, distance_mm);
        }
        imshow("Detection Result", result_img);

        int key = waitKey(30);
        if (key == 27) break;
        if (key == 's') {
            cout << "Current BGR Settings:\n";
            cout << "Blue Threshold: " << blue_thresh << endl;
            cout << "Blue Boost: " << blue_boost << endl;
        }
    }

    destroyAllWindows();
    return 0;
}
