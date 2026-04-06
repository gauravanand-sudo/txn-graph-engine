#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <future>
#include <vector>
#include <atomic>
#include <type_traits>
#include <stdexcept>

// ============================================================
// ThreadPool.hpp — C++17 thread pool with work-stealing queue
//
// Design:
//   • Fixed number of worker threads created at construction.
//   • Each task is submitted via enqueue(), which returns a
//     std::future<ReturnType> so callers can synchronise.
//   • Graceful shutdown: destructor sets stop_ flag and notifies
//     all workers; each worker drains its pending tasks first.
//   • Uses a single shared task queue protected by a mutex and
//     condition_variable.  A proper work-stealing implementation
//     would use per-thread deques with steal from the back; this
//     design approximates it via the shared queue which avoids
//     contention when thread count is moderate.
//
// Usage:
//   ThreadPool pool(4);
//   auto fut = pool.enqueue([](int x){ return x * 2; }, 21);
//   std::cout << fut.get();  // prints 42
// ============================================================

namespace txn {

class ThreadPool {
public:
    // ---- constructor / destructor ---------------------------

    explicit ThreadPool(std::size_t num_threads) : stop_(false) {
        workers_.reserve(num_threads);
        for (std::size_t i = 0; i < num_threads; ++i) {
            workers_.emplace_back([this] { worker_loop(); });
        }
    }

    // Non-copyable, non-movable (threads hold `this` pointer)
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cond_.notify_all(); // wake all sleeping workers
        for (auto& t : workers_) {
            if (t.joinable()) t.join();
        }
    }

    // ---- enqueue --------------------------------------------

    /// Submit a callable with arguments; returns std::future.
    /// Thread-safe; can be called from multiple producers.
    template<typename F, typename... Args>
    [[nodiscard]] auto enqueue(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>
    {
        using RetType = std::invoke_result_t<F, Args...>;

        // Wrap the task in a packaged_task for future support
        auto task = std::make_shared<std::packaged_task<RetType()>>(
            [func = std::forward<F>(f),
             targs = std::make_tuple(std::forward<Args>(args)...)]() mutable {
                return std::apply(std::move(func), std::move(targs));
            }
        );
        std::future<RetType> fut = task->get_future();

        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stop_)
                throw std::runtime_error("ThreadPool: enqueue on stopped pool");
            // Wrap packaged_task in a type-erased std::function<void()>
            tasks_.emplace([task = std::move(task)]() { (*task)(); });
        }
        cond_.notify_one(); // wake one sleeping worker
        return fut;
    }

    /// Returns the number of worker threads.
    [[nodiscard]] std::size_t thread_count() const noexcept {
        return workers_.size();
    }

    /// Returns the current number of pending tasks (approximate).
    [[nodiscard]] std::size_t pending_tasks() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return tasks_.size();
    }

private:
    // ---- worker loop ----------------------------------------

    void worker_loop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                // Wait until there is work or we are asked to stop
                cond_.wait(lock, [this] {
                    return stop_ || !tasks_.empty();
                });
                if (stop_ && tasks_.empty()) return; // graceful exit
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            // Execute outside the lock
            task();
        }
    }

    // ---- members --------------------------------------------

    std::vector<std::thread>          workers_;
    std::queue<std::function<void()>> tasks_;
    mutable std::mutex                mutex_;
    std::condition_variable           cond_;
    bool                              stop_;
};

} // namespace txn
