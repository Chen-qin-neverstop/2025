#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(value);
        cond_.notify_one();
    }

    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        value = queue_.front();
        queue_.pop();
        return true;
    }

    void wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return !queue_.empty(); });
        value = queue_.front();
        queue_.pop();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable cond_;
};

#endif // THREAD_SAFE_QUEUE_H