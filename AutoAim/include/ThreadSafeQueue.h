#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(value);
        lock.unlock();
        cond_.notify_one();
    }
    
    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty() || !running_) {
            return false;
        }
        value = queue_.front();
        queue_.pop();
        return true;
    }
    
    bool wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] { return !queue_.empty() || !running_; });
        if (!running_) return false;
        value = queue_.front();
        queue_.pop();
        return true;
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
    }
    
    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        cond_.notify_all();
    }
    
private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable cond_;
    std::atomic<bool> running_{true};
};

#endif // THREAD_SAFE_QUEUE_H