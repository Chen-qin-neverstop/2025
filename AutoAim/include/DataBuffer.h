#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief 线程安全队列模板类
 * @tparam T 存储元素类型
 * 
 * 特性：
 * 1. 支持阻塞和非阻塞两种获取方式
 * 2. 内置停止机制可安全终止所有等待线程
 * 3. 移动语义优化性能
 */
template<typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;
    ~ThreadSafeQueue() = default;

    // 禁止拷贝和赋值
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    /**
     * @brief 将元素推入队列
     * @param value 要添加的元素（自动使用完美转发）
     */
    void push(T&& value);

    /**
     * @brief 尝试从队列弹出元素（非阻塞）
     * @param value 弹出元素的存储位置
     * @return 是否成功弹出
     */
    bool try_pop(T& value);

    /**
     * @brief 等待并弹出元素（阻塞式）
     * @param value 弹出元素的存储位置
     * @throws std::runtime_error 当队列被停止时抛出
     */
    void wait_and_pop(T& value);

    /**
     * @brief 安全停止队列，唤醒所有等待线程
     */
    void stop();

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable cond_;
    bool stop_flag_ = false;
};

/**
 * @brief 双缓冲类（专为图像帧设计）
 * 
 * 典型工作流程：
 * 1. 生产者向back buffer填充数据
 * 2. 消费者交换缓冲区后处理front buffer
 * 3. 重复上述过程
 */

class DoubleBuffer {
public:
    bool try_push(cv::Mat&& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        int back_index = 1 - front_buffer_index_;  // 动态计算后端索引
        if (buffers_[back_index].size() >= max_buffer_size_) {
            return false;
        }
        buffers_[back_index].emplace_back(std::move(frame));
        new_data_available_ = true;
        return true;
    }

    std::vector<cv::Mat> get_front_buffer() {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::move(buffers_[front_buffer_index_]);
    }

    void swap_buffers() {
        std::lock_guard<std::mutex> lock(mutex_);
        front_buffer_index_ = 1 - front_buffer_index_;
        new_data_available_ = false;
    }

    bool wait_for_data(unsigned timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cond_.wait_for(lock, timeout_ms * 1ms, 
            [this] { return new_data_available_ || stop_flag_; });
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_flag_ = true;
        }
        cond_.notify_all();
    }

private:
    std::vector<cv::Mat> buffers_[2];
    int front_buffer_index_ = 0;  // 只需存储前端索引
    const size_t max_buffer_size_ = 10;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::atomic<bool> new_data_available_{false};
    std::atomic<bool> stop_flag_{false};
};
// 帧数据结构（时间戳+帧数据）
struct FrameData {
    cv::Mat frame;
    int frame_count = 0;
    double timestamp = 0.0;   

    // 移动构造优化性能
    FrameData() = default;
    FrameData(FrameData&&) = default;
    FrameData& operator=(FrameData&&) = default;

    // 禁止拷贝
    FrameData(const FrameData&) = delete;
    FrameData& operator=(const FrameData&) = delete;
};

// 显式实例化声明（实现在.cpp中）
extern template class ThreadSafeQueue<FrameData>;
extern template class ThreadSafeQueue<cv::Point3f>;

#endif // DATA_BUFFER_H