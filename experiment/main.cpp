#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"
#include "CoordinateTransformer.h" // 位姿转换
#include "KalmanFilter.h" // 卡尔曼滤波

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    // 直接指定图片路径
    std::string image_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/18.jpg";
    Mat img = imread(image_path);
    if (img.empty()) {
        cerr << "无法读取图片: " << image_path << endl;
        return -1;
    }

    // 1. 图像识别：检测装甲板
    Mat binary = preprocessImage(img);
    imshow("binary", binary);
    imwrite("binary_result.jpg", binary);

    vector<RotatedRect> light_bars = findLightBars(binary);
    // 可视化灯条
    Mat light_bar_vis = img.clone();
    for (const auto& rect : light_bars) {
        Point2f pts[4];
        rect.points(pts);
        for (int i = 0; i < 4; ++i)
            line(light_bar_vis, pts[i], pts[(i+1)%4], Scalar(255,0,0), 2);
    }
    imshow("light_bars", light_bar_vis);
    imwrite("light_bars_result.jpg", light_bar_vis);

    auto armor_pairs = matchArmorPairs(light_bars);
    // 可视化装甲板配对
    Mat armor_pair_vis = img.clone();
    for (const auto& pair : armor_pairs) {
        Point2f c1 = pair.first.center;
        Point2f c2 = pair.second.center;
        line(armor_pair_vis, c1, c2, Scalar(0,0,255), 2);
        circle(armor_pair_vis, c1, 4, Scalar(0,255,255), -1);
        circle(armor_pair_vis, c2, 4, Scalar(0,255,255), -1);
    }
    imshow("armor_pairs", armor_pair_vis);
    imwrite("armor_pairs_result.jpg", armor_pair_vis);

    if (armor_pairs.empty()) {
        cout << "未检测到装甲板" << endl;
        return 0;
    }

    // 只取第一个装甲板对
    auto armor_corners = getArmorCorners(armor_pairs[0]);

    // 2. 位姿转换：像素坐标->世界坐标（PnP解算）
    Mat rvec, tvec;
    solveArmorPose(armor_corners, rvec, tvec);

    // 3. 构造Pose对象
    // rvec为旋转向量，需转欧拉角
    Mat R;
    Rodrigues(rvec, R);
    // 旋转矩阵转欧拉角
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0));
    double roll, pitch, yaw;
    if (sy > 1e-6) {
        roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = 0;
    }
    Pose pose(
        tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0),
        roll, pitch, yaw
    );

    // 4. 卡尔曼滤波
    ::KalmanFilter kf(6, 3, 0);
    cv::Mat initial_state = (cv::Mat_<float>(6,1) <<
        tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0), 0, 0, 0);
    kf.init(initial_state);
    kf.predict();
    cv::Mat measurement = (cv::Mat_<float>(3,1) <<
        tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
    cv::Mat filtered = kf.correct(measurement);

    // 5. 输出结果
    cout << "检测到装甲板，原始位姿: ";
    pose.print();
    cout << endl;
    cout << "滤波后位置: [" << filtered.at<float>(0,0) << ", "
         << filtered.at<float>(1,0) << ", " << filtered.at<float>(2,0) << "]" << endl;

    // 6. 可视化
    for (const auto& pt : armor_corners) {
        circle(img, pt, 4, Scalar(0,255,0), -1);
    }
    Point2f center = (armor_corners[0] + armor_corners[2]) * 0.5f;
    drawDistanceInfo(img, center, norm(tvec));
    imshow("result", img);
    waitKey(0);

    return 0;
} 
