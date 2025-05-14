# RM Vision 2025

HDU RM Phoenix 2025 视觉培训项目

## 项目结构

- `AutoAim/`: 自瞄系统核心代码
- `Armor_detector/`: 装甲板检测模块
- `CoordinateTransformer/`: 坐标转换模块
- `Kalman/`: 卡尔曼滤波模块
- `config/`: 配置文件目录

## 依赖要求

- CMake >= 3.10
- OpenCV >= 4.2.0
- Eigen3 >= 3.3.0
- C++11 或更高版本

## 构建说明

1. 安装依赖：

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    libeigen3-dev
```

2. 构建项目：

```bash
mkdir build && cd build
cmake ..
make -j4
```

3. 运行：

```bash
# 运行服务器
./bin/armor_server

# 运行客户端
./bin/armor_client
```

## 配置说明

配置文件位于 `config/` 目录下：
- `camera.yaml`: 相机参数配置
- 其他配置文件...

## 开发说明

1. 代码风格遵循 Google C++ Style Guide
2. 提交代码前请确保通过所有测试
3. 新功能开发请创建新的分支
