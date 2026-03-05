#ifndef QCONTROLLER_WORKER
#define QCONTROLLER_WORKER

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

/**
 * @brief Executes tasks in a separate thread, used with Qt for large tasks that woudl slow down the gui
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

    std::shared_ptr<rclcpp::Node> _node;

    std::thread _executorThread;
    std::mutex _queueMutex;
    std::atomic<bool> _running{true};
    std::condition_variable _cv;
    std::queue<std::function<void()>> _queue;
};

#endif  // QCONTROLLER_WORKER