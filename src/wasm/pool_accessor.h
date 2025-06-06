#pragma once
#include "thread_pool.h"

inline ThreadPool& getThreadPool() {
    static ThreadPool pool(
        std::max(2u, std::thread::hardware_concurrency())); // or 固定値
    return pool;
}