#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageProcess.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    std::string image_path = "/home/chen/Project/Vscode/Code/AutoAIM/2025/experiment/18.jpg";
    Mat img = imread(image_path);
    if (img.empty()) {
        cerr << "无法读取图片: " << image_path << endl;
        return -1;
    }

    Mat binary = preprocessImage(img);
    vector<RotatedRect> light_bars = findLightBars(binary);
    
    vector<pair<RotatedRect, RotatedRect>> armor_pairs = matchArmorPairs(light_bars);
    Mat armor_pair_vis = img.clone();

    if (armor_pairs.empty()) {
        cout << "未检测到装甲板" << endl;
        return 0;
    }

    vector<Point2f> armor_corners = getArmorCorners(armor_pairs[0]);
    Mat rvec, tvec;
    solveArmorPose(armor_corners, rvec, tvec);

    Point2f center = (armor_corners[0] + armor_corners[2]) * 0.5f;
    drawDistanceInfo(img, norm(tvec),armor_corners);
    imshow("result", img);
    waitKey(0);
    return 0;
}
