# AutoAim 自动瞄准系统

这是一个基于计算机视觉的自动瞄准系统，包含服务器端（装甲板检测）和客户端（视频采集和显示）两个部分。

## 功能特点

- 实时装甲板检测和识别
- 3D姿态估计
- 运动状态预测
- 实时视频显示和可视化
- 支持视频文件和摄像头输入

## 系统要求

- Ubuntu 20.04 或更高版本
- OpenCV 4.x
- CMake 3.10 或更高版本
- C++17 或更高版本
- GCC 7.5 或更高版本

## 编译方法

1. 克隆仓库：
```bash
git clone <repository_url>
cd AutoAim
```

2. 创建并进入构建目录：
```bash
mkdir build
cd build
```

3. 配置和编译：
```bash
cmake ..
make -j4
```

编译完成后，在 `build` 目录下会生成两个可执行文件：
- `armor_detection_server`：服务器端程序
- `video_client`：客户端程序

## 运行方法

### 1. 启动服务器

首先启动服务器端程序：
```bash
./armor_detection_server [port]
```
- `port`：可选参数，指定服务器端口号，默认为 8080

### 2. 启动客户端

然后启动客户端程序：
```bash
./video_client [server_ip] [server_port] [num_threads]
```
- `server_ip`：可选参数，服务器IP地址，默认为 "127.0.0.1"
- `server_port`：可选参数，服务器端口号，默认为 8080
- `num_threads`：可选参数，工作线程数，默认为 4

例如：
```bash
# 使用默认参数
./video_client

# 指定服务器IP和端口
./video_client 192.168.1.100 8080

# 指定所有参数
./video_client 192.168.1.100 8080 8
```

### 3. 视频源选择

客户端程序会按以下顺序尝试打开视频源：
1. 首先尝试打开视频文件（默认路径：`/home/chen/视频/video/videos/2.mp4`）
2. 如果视频文件打开失败，会尝试打开摄像头（设备号 0 和 1）

### 4. 操作说明

- 程序运行后，会显示一个视频窗口，实时显示视频流和装甲板检测结果
- 装甲板检测结果会以绿色框和红色角点标记显示
- 按 `ESC` 键退出程序

## 常见问题

1. 如果无法打开视频文件：
   - 检查视频文件路径是否正确
   - 确保视频文件格式支持（建议使用 MP4 格式）

2. 如果无法打开摄像头：
   - 检查摄像头是否正确连接
   - 确保没有其他程序占用摄像头
   - 检查用户是否有摄像头访问权限

3. 如果无法连接到服务器：
   - 检查服务器是否正在运行
   - 检查 IP 地址和端口号是否正确
   - 检查防火墙设置

4. 如果服务器启动时报 "Bind failed" 错误：
   - 检查端口是否被占用：
     ```bash
     sudo lsof -i :8080
     # 或
     sudo netstat -tulpn | grep 8080
     ```
   - 解决方案：
     - 关闭占用端口的程序：
       ```bash
       sudo kill <PID>  # 替换 <PID> 为占用端口的进程ID
       ```
     - 或使用不同的端口：
       ```bash
       # 服务器端
       ./armor_detection_server 8081
       
       # 客户端
       ./video_client 127.0.0.1 8081
       ```

## 目录结构

```
AutoAim/
├── CMakeLists.txt
├── include/
│   ├── Protocol.h
│   ├── VideoClient.h
│   └── ArmorDetectionServer.h
├── src/
│   ├── client/
│   │   └── VideoClient.cpp
│   └── server/
│       ├── ArmorDetectionServer.cpp
│       └── Protocol.cpp
└── README.md
```

## 注意事项

1. 确保服务器端先启动，再启动客户端
2. 如果使用摄像头，建议先关闭其他可能占用摄像头的程序
3. 程序运行时需要足够的系统资源，建议关闭不必要的后台程序
4. 如果遇到性能问题，可以调整工作线程数（num_threads 参数）

## 许可证

[添加许可证信息]

## 联系方式

[添加联系方式]
