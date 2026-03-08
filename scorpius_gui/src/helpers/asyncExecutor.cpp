#include "asyncExecutor.hpp"

AsyncExecutor::AsyncExecutor()
{
    _executorThread = std::thread(&AsyncExecutor::executorLoop, this);
}

AsyncExecutor::~AsyncExecutor()
{
    this->stop();
}

void AsyncExecutor::stop()
{
    bool expected = true;

    // Returns true if _running is expected (true) and starts shutdown
    // Returns false if _running was already false (already stopped), bail out

    if (!_running.compare_exchange_strong(expected, false))
    {
        return;  // bail out
    }

    _cv.notify_all();

    if (_executorThread.joinable())
    {
        _executorThread.join();
    }
}

void AsyncExecutor::executorLoop()
{
    while (_running)
    {
        std::function<void(void)> task;

        {
            std::unique_lock<std::mutex> lock(_queueMutex);

            _cv.wait(lock,
                     [this](void)
                     {
                         return !this->_queue.empty() || !this->_running;
                     });

            if (!_running && _queue.empty())
            {
                break;
            }

            task = std::move(_queue.front());
            _queue.pop();
        }

        if (task)
        {
            task();
        }
    }
}

void AsyncExecutor::addTask(std::function<void()> task_)
{
    if (!task_)
    {
        return;
    }

    {
        std::lock_guard lock(_queueMutex);
        _queue.push(std::move(task_));
    }

    _cv.notify_one();
}