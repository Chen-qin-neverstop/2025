#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

// ====================== 四元数类 (基于OpenCV) ======================
class Quaternion {
private:
    cv::Vec4d data; // 使用OpenCV的Vec4d存储四元数(w, x, y, z)

public:
    // 默认构造函数，初始化为单位四元数
    Quaternion() : data(1.0, 0.0, 0.0, 0.0) {}

    // 带参数的构造函数
    Quaternion(double w, double x, double y, double z) : data(w, x, y, z) {}

    // 从旋转矩阵构造四元数
    Quaternion(const cv::Mat& rotMat) {
        // 使用OpenCV的旋转矩阵到四元数转换
        cv::Vec4d q;
        cv::Rodrigues(rotMat, q);
        data = cv::Vec4d(q[3], q[0], q[1], q[2]); // OpenCV返回的是(x,y,z,w)
    }

    // 从欧拉角构造四元数 (绕Z-Y-X顺序旋转，即yaw-pitch-roll)
    Quaternion(double roll, double pitch, double yaw) {
        // 使用OpenCV的旋转向量表示
        cv::Mat rotVec = (cv::Mat_<double>(3,1) << roll, pitch, yaw);
        cv::Mat rotMat;
        cv::Rodrigues(rotVec, rotMat);
        
        // 从旋转矩阵获取四元数
        cv::Vec4d q;
        cv::Rodrigues(rotMat, q);
        data = cv::Vec4d(q[3], q[0], q[1], q[2]); // OpenCV返回的是(x,y,z,w)
    }

    // 获取四元数的范数(长度)
    double norm() const {
        return cv::norm(data);
    }

    // 四元数归一化
    Quaternion normalized() const {
        double n = norm();
        if (n == 0.0) return Quaternion();
        return Quaternion(data[0]/n, data[1]/n, data[2]/n, data[3]/n);
    }

    // 四元数共轭
    Quaternion conjugate() const {
        return Quaternion(data[0], -data[1], -data[2], -data[3]);
    }

    // 四元数乘法 (用于旋转组合)
    Quaternion operator*(const Quaternion& q) const {
        // 转换为旋转矩阵相乘
        cv::Mat rotMat1 = this->toRotationMatrix();
        cv::Mat rotMat2 = q.toRotationMatrix();
        cv::Mat resultRotMat = rotMat2 * rotMat1; // OpenCV是列优先
        
        // 转换回四元数
        return Quaternion(resultRotMat);
    }

    // 将四元数转换为旋转矩阵
    cv::Mat toRotationMatrix() const {
        // OpenCV期望四元数为(x,y,z,w)
        cv::Vec4d q(data[1], data[2], data[3], data[0]);
        cv::Mat rotMat;
        cv::Rodrigues(q, rotMat);
        return rotMat;
    }

    // 将四元数转换为欧拉角 (roll, pitch, yaw)
    cv::Vec3d toEulerAngles() const {
        cv::Mat rotMat = this->toRotationMatrix();
        
        // 从旋转矩阵提取欧拉角
        double sy = sqrt(rotMat.at<double>(0,0) * rotMat.at<double>(0,0) + 
                      rotMat.at<double>(1,0) * rotMat.at<double>(1,0));
        
        bool singular = sy < 1e-6;
        
        double x, y, z;
        if (!singular) {
            x = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
            y = atan2(-rotMat.at<double>(2,0), sy);
            z = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
        } else {
            x = atan2(-rotMat.at<double>(1,2), rotMat.at<double>(1,1));
            y = atan2(-rotMat.at<double>(2,0), sy);
            z = 0;
        }
        
        return cv::Vec3d(x, y, z);
    }

    // 打印四元数
    void print() const {
        std::cout << "Quaternion(" << data[0] << ", " << data[1] 
                  << ", " << data[2] << ", " << data[3] << ")" << std::endl;
    }

    // 获取四元数分量
    double getW() const { return data[0]; }
    double getX() const { return data[1]; }
    double getY() const { return data[2]; }
    double getZ() const { return data[3]; }
};

// ====================== 位姿类 (基于OpenCV) ======================
class Pose {
private:
    cv::Vec3d position;  // 位置 (x, y, z)
    cv::Vec3d orientation; // 欧拉角姿态 (roll, pitch, yaw)

public:
    // 默认构造函数
    Pose() : position(0.0, 0.0, 0.0), orientation(0.0, 0.0, 0.0) {}

    // 带参数的构造函数
    Pose(double x, double y, double z, double roll, double pitch, double yaw) 
        : position(x, y, z), orientation(roll, pitch, yaw) {}

    // 从位置和四元数构造位姿
    Pose(const cv::Vec3d& pos, const Quaternion& q) : position(pos) {
        orientation = q.toEulerAngles();
    }

    // 获取位置
    cv::Vec3d getPosition() const {
        return position;
    }

    // 获取欧拉角姿态
    cv::Vec3d getOrientation() const {
        return orientation;
    }

    // 获取四元数表示的姿态
    Quaternion getQuaternion() const {
        return Quaternion(orientation[0], orientation[1], orientation[2]);
    }

    // 获取变换矩阵 (4x4齐次变换矩阵)
    cv::Mat getTransformationMatrix() const {
        Quaternion q = this->getQuaternion();
        cv::Mat rotMat = q.toRotationMatrix();
        
        cv::Mat transMat = cv::Mat::eye(4, 4, CV_64F);
        rotMat.copyTo(transMat(cv::Rect(0, 0, 3, 3)));
        transMat.at<double>(0, 3) = position[0];
        transMat.at<double>(1, 3) = position[1];
        transMat.at<double>(2, 3) = position[2];
        
        return transMat;
    }

    // 打印位姿信息
    void print() const {
        std::cout << "Position: (" << position[0] << ", " 
                  << position[1] << ", " << position[2] << ")\n";
        std::cout << "Orientation (roll, pitch, yaw): (" 
                  << orientation[0] << ", " << orientation[1] 
                  << ", " << orientation[2] << ")\n";
    }
};

// ====================== 坐标转换函数 (基于OpenCV) ======================
/**
 * @brief 将位姿从源坐标系转换到目标坐标系
 * @param pose 源坐标系下的位姿
 * @param sourceFrame 源坐标系相对于目标坐标系的位姿
 * @return 目标坐标系下的位姿
 */
Pose transformPose(const Pose& pose, const Pose& sourceFrame) {
    // 获取变换矩阵
    cv::Mat poseMat = pose.getTransformationMatrix();
    cv::Mat sourceMat = sourceFrame.getTransformationMatrix();
    
    // 计算目标坐标系下的变换矩阵
    cv::Mat targetMat = sourceMat * poseMat;
    
    // 提取位置
    cv::Vec3d position(
        targetMat.at<double>(0, 3),
        targetMat.at<double>(1, 3),
        targetMat.at<double>(2, 3)
    );
    
    // 提取旋转矩阵并转换为欧拉角
    cv::Mat rotMat = targetMat(cv::Rect(0, 0, 3, 3));
    Quaternion q(rotMat);
    cv::Vec3d orientation = q.toEulerAngles();
    
    return Pose(position, q);
}

// ====================== 主函数测试 ======================
int main() {
    // 测试四元数类
    std::cout << "===== Quaternion Test =====" << std::endl;
    Quaternion q1(0.707, 0.0, 0.707, 0.0); // 绕y轴旋转90度
    q1.print();
    cv::Vec3d euler = q1.toEulerAngles();
    std::cout << "Euler angles (roll, pitch, yaw): " 
              << euler[0] << ", " << euler[1] << ", " << euler[2] << std::endl;
    
    // 测试从欧拉角构造四元数
    Quaternion q2(CV_PI/4, CV_PI/6, CV_PI/3); // roll=45°, pitch=30°, yaw=60°
    q2.print();
    
    // 测试位姿类
    std::cout << "\n===== Pose Test =====" << std::endl;
    Pose p1(1.0, 2.0, 3.0, CV_PI/4, CV_PI/6, CV_PI/3);
    p1.print();
    
    // 测试坐标转换
    std::cout << "\n===== Coordinate Transformation Test =====" << std::endl;
    // 定义源坐标系相对于目标坐标系的位姿
    // 假设源坐标系相对于目标坐标系在x方向偏移2，y方向偏移1，z方向偏移0.5
    // 并且绕z轴旋转30度
    Pose sourceFrame(2.0, 1.0, 0.5, 0.0, 0.0, CV_PI/6);
    std::cout << "Source frame relative to target frame:" << std::endl;
    sourceFrame.print();
    
    // 定义源坐标系下的一个位姿
    Pose poseInSource(1.0, 0.5, 0.0, CV_PI/4, 0.0, 0.0);
    std::cout << "\nPose in source frame:" << std::endl;
    poseInSource.print();
    
    // 进行坐标转换
    Pose poseInTarget = transformPose(poseInSource, sourceFrame);
    std::cout << "\nTransformed pose in target frame:" << std::endl;
    poseInTarget.print();
    
    return 0;
}
