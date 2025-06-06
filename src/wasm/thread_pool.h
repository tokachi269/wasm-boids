#pragma once
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
public:
    explicit ThreadPool(std::size_t numThreads);
    ~ThreadPool();

    template <class F>
    auto enqueue(F&& f) -> std::future<decltype(f())>;

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;

    std::mutex              mtx_;
    std::condition_variable cv_;
    bool                    stop_ = false;
};

// thread_pool.cpp  —  いわゆる典型的実装
#include "thread_pool.h"
ThreadPool::ThreadPool(std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        workers_.emplace_back([this]{
            for (;;) {
                std::function<void()> task;
                {
                    std::unique_lock lk(mtx_);
                    cv_.wait(lk, [this]{ return stop_ || !tasks_.empty(); });
                    if (stop_ && tasks_.empty()) return;
                    task = std::move(tasks_.front());
                    tasks_.pop();
                }
                task();
            }
        });
    }
}
ThreadPool::~ThreadPool() {
    {
        std::lock_guard lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& w : workers_) w.join();
}
template <class F>
auto ThreadPool::enqueue(F&& f) -> std::future<decltype(f())> {
    using Ret = decltype(f());
    auto p = std::make_shared<std::packaged_task<Ret()>>(std::forward<F>(f));
    std::future<Ret> fut = p->get_future();
    {
        std::lock_guard lk(mtx_);
        tasks_.emplace([p]{ (*p)(); });
    }
    cv_.notify_one();
    return fut;
}