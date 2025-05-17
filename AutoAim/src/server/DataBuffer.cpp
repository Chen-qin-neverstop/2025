#include "DataBuffer.h"
#include <chrono>

using namespace std::chrono_literals;

// ================= ThreadSafeQueue 实现 =================
template<typename T>
void ThreadSafeQueue<T>::push(T&& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(std::move(value));  // 使用移动语义
    lock.unlock();
    cond_.notify_one();
}

template<typename T>
bool ThreadSafeQueue<T>::try_pop(T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
        return false;
    }
    value = std::move(queue_.front());  // 使用移动语义
    queue_.pop();
    return true;
}

template<typename T>
void ThreadSafeQueue<T>::wait_and_pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this] { return !queue_.empty() || stop_flag_; });
    if (stop_flag_) {
        throw std::runtime_error("Queue stopped");
    }
    value = std::move(queue_.front());  // 使用移动语义
    queue_.pop();
}

template<typename T>
void ThreadSafeQueue<T>::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_ = true;
    }
    cond_.notify_all();
}

// 显式实例化模板（包含新增的FrameData）
template class ThreadSafeQueue<FrameData>;
template class ThreadSafeQueue<cv::Point3f>;

