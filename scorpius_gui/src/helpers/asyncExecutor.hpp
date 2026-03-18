#ifndef ASYNC_EXECUTOR_HPP
#define ASYNC_EXECUTOR_HPP

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

/**
 * @brief Executes tasks in a separate thread, used with Qt for large tasks that would slow down the gui
 * is not dependent on rclcpp or qt so can be reused anywhere
 *
 */
class AsyncExecutor
{
  public:
    AsyncExecutor();
    ~AsyncExecutor();
    void addTask(std::function<void(void)> task_);
    void stop();

  private:
    void executorLoop();

    std::thread _executorThread;
    std::mutex _queueMutex;
    std::atomic<bool> _running{true};
    std::condition_variable _cv;
    std::queue<std::function<void()>> _queue;
};

#endif  // ASYNC_EXECUTOR_HPP