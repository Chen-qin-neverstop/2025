// #include "CoordinateTransformer.h"
// #include <iostream>
// #include <vector>
// #include <sstream>
// #include <iomanip>

// // // 整合输入输出的转换函数
// // std::vector<double> transformCoordinates(
// //     double x, double y, double z,
// //     double roll, double pitch, double yaw,
// //     const std::string& target_frame) 
// // {
// //     try {
// //         // 创建位姿对象
// //         Pose camera_pose(x, y, z, roll, pitch, yaw);
        
// //         // 执行坐标系转换
// //         CoordinateTransformer transformer;
// //         Pose target_pose = transformer.transformToTarget(camera_pose, target_frame);
        
// //         // 获取转换后的数据
// //         std::vector<double> result;
// //         auto pos = target_pose.getPosition();
// //         auto orient = target_pose.getOrientation();
        
// //         result.insert(result.end(), pos.begin(), pos.end());
// //         result.insert(result.end(), orient.begin(), orient.end());
        
// //         return result;
        
// //     } catch (const std::exception& e) {
// //         std::cerr << "转换错误: " << e.what() << std::endl;
// //         return {};
// //     }
// // }

// std::vector<double> transformCoordinates(
//     double x, double y, double z,
//     double yaw, double pitch, double roll,  // 输入顺序改为yaw,pitch,roll
//     const std::string& target_frame) 
// {
//     Pose camera_pose(x, y, z, yaw, pitch, roll);
//     CoordinateTransformer transformer;
//     Pose target_pose = transformer.transformToTarget(camera_pose, target_frame);
    
//     // 输出顺序保持[yaw,pitch,roll]
//     auto pos = target_pose.getPosition();
//     auto orient = target_pose.getOrientation();
    
//     return {
//         pos[0], pos[1], pos[2],    // x,y,z
//         orient[0], orient[1], orient[2]  // yaw,pitch,roll
//     };
// }

// // int main() {
// //     // 示例
// //     double x = 1.0, y = 2.0, z = 3.0;
// //     double roll = 0.1, pitch = 0.1, yaw = 0.1;
// //     std::string target_frame = "/Odom";
    
// //     // 调用转换函数
// //     auto result = transformCoordinates(x, y, z, yaw, pitch, roll, target_frame);
    
// //     // 输出结果
// //     if (!result.empty()) {
// //         std::cout << "转换结果: ";
// //         for (size_t i = 0; i < result.size(); ++i) {
// //             std::cout << result[i];
// //             if (i != result.size() - 1) std::cout << " ";
// //         }
// //         std::cout << std::endl;
// //     }
    
// //     return 0;
// // }


#include "CoordinateTransformer.h"
#include <iostream>
#include <iomanip>
#include <vector>

std::vector<double> transformCoordinates(
    double x, double y, double z,
    double yaw, double pitch, double roll,
    const std::string& target_frame) 
{
    Pose camera_pose(x, y, z, yaw, pitch, roll);
    CoordinateTransformer transformer;
    Pose target_pose = transformer.transformToTarget(camera_pose, target_frame);
    
    auto pos = target_pose.getPosition();
    auto orient = target_pose.getOrientation();
    return {pos[0], pos[1], pos[2], orient[0], orient[1], orient[2]};
}

// int main() {
//     auto result = transformCoordinates(
//         1.0, 2.0, 3.0,    // x,y,z
//         0.1, 0.1, 0.1,     // yaw,pitch,roll
//         "/Odom"            // 目标坐标系
//     );

//     std::cout << std::fixed << std::setprecision(6);
//     std::cout << "转换结果: "
//               << result[0] << " " << result[1] << " " << result[2] << " "
//               << result[3] << " " << result[4] << " " << result[5] 
//               << std::endl;
    
//     return 0;
// }


int main() {
    try {
        // 测试转换到Gimbal系
        auto result_gimbal = transformCoordinates(
            1.0, 2.0, 3.0,     // Camera系下的位置
            0.3, 0.2, 0.1,      // Camera系下的旋转 (yaw,pitch,roll)
            "/Gimbal"           // 目标坐标系
        );
        std::cout << "Camera→Gimbal: ";
        std::cout << result_gimbal[0] << " " << result_gimbal[1] << " " << result_gimbal[2] << " "
                  << result_gimbal[3] << " " << result_gimbal[4] << " " << result_gimbal[5] << std::endl;

        // 测试转换到Odom系
        auto result_odom = transformCoordinates(
            1.0, 2.0, 3.0,
            0.3, 0.2, 0.1,
            "/Odom"
        );
        std::cout << "Camera→Odom: ";
        std::cout << result_odom[0] << " " << result_odom[1] << " " << result_odom[2] << " "
                  << result_odom[3] << " " << result_odom[4] << " " << result_odom[5] << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
