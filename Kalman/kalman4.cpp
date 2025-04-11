#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

class KalmanFilter {
public:
// 卡尔曼滤波器参数
    // Q: 过程噪声协方差，R: 测量噪声协方差
    // estimate: 当前估计值，error: 当前估计误差
    KalmanFilter(double q, double r, double initial_estimate, double initial_error)
        : Q(q), R(r), estimate(initial_estimate), error(initial_error) {}
// 更新卡尔曼滤波器
    // measurement: 当前测量值
    // 返回滤波后的估计值
    // 计算卡尔曼增益K
    // 更新估计值和误差
    // 计算新的估计值和误差
    // 返回滤波后的估计值
    // 更新卡尔曼滤波器
    double update(double measurement) {
        error += Q;
        double K = error / (error + R);
        estimate += K * (measurement - estimate);
        error *= (1 - K);
        return estimate;
    }

    double getEstimate() const { return estimate; }

private:
    double Q, R, estimate, error;
};

int main() {
    std::vector<double> measurements;
    std::ifstream file("homework_data_4.txt");
    std::string line;

    // 读取数据
    while (std::getline(file, line)) {
        double time, value;
        std::istringstream iss(line);
        if (iss >> time >> value) {
            measurements.push_back(value);
        }
    }

    // 初始化卡尔曼滤波器
    KalmanFilter kf(0.0001, 0.5, measurements[0], 0.01);
// 目前最好0.1 0.5 1.0
    // 输出原始数据和滤波结果到文件（用于绘图）
    std::ofstream out("kalman_data5.txt");
    out << "Time\tMeasurement\tEstimate\n";
    for (size_t i = 0; i < measurements.size(); ++i) {
        double estimate = kf.update(measurements[i]);
        out << i * 0.001 << "\t" << measurements[i] << "\t" << estimate << "\n");
    }
    out.close();

    // 输出拟合方程（卡尔曼滤波的最终状态）
    std::cout << "Fitted Curve Equation:\n";
    std::cout << "y(t) = " << kf.getEstimate() << " (smoothed by Kalman Filter)\n";

    return 0;
}
