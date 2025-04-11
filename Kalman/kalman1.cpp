#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

class KalmanFilter {
public:
    KalmanFilter(double q, double r, double initial_estimate, double initial_error)
        : Q(q), R(r), estimate(initial_estimate), error(initial_error) {}

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
    std::ifstream file("homework_data_1.txt");
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
    KalmanFilter kf(0.001, 0.5, measurements[0], 0.05);
// q: 过程噪声协方差  表示系统模型的不确定性。值越大，滤波器对新测量值的信任度越高，更新时会更快地调整估计值。 ◦ 
// r: 测量噪声协方差  表示测量值的不确定性。值越大，滤波器对测量值的信任度越低，更新时会更慢地调整估计值。 ◦
// initial_estimate: 初始估计值  表示滤波器对系统状态的初始猜测。通常使用第一个测量值作为初始估计值。 ◦
// initial_error: 初始误差协方差    表示滤波器对初始估计值的不确定性。值越大，滤波器对初始估计值的信任度越低，更新时会更慢地调整估计值。
// 这里设置为0.05，表示对初始估计值有一定的不确定性。
// 为了让滤波器更快地收敛，将其设置为一个较小的值，例如0.01或0.001。
// 这将使滤波器对新测量值的信任度更高，从而更快地调整估计值。
// 过程噪声协方差（Q）和测量噪声协方差（R）可以根据实际情况进行调整。一般来说，Q和R的值越小，滤波器对测量值的信任度越高，更新时会更快地调整估计值；反之，则会更慢地调整估计值。

    // 输出原始数据和滤波结果到文件（用于绘图）
    std::ofstream out("kalman_data.txt");
    out << "Time\tMeasurement\tEstimate\n";
    for (size_t i = 0; i < measurements.size(); ++i) {
        double estimate = kf.update(measurements[i]);
        out << i * 0.001 << "\t" << measurements[i] << "\t" << estimate << "\n";
    }
    out.close();

    // 输出拟合方程（卡尔曼滤波的最终状态）
    std::cout << "Fitted Curve Equation:\n";
    std::cout << "y(t) = " << kf.getEstimate() << " (smoothed by Kalman Filter)\n";

    return 0;
}
