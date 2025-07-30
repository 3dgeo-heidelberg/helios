#pragma once

#include <ThreadPool.h>
#include <boost/asio/post.hpp>
/**
 * @version 1.0
 * @brief Abstract class providing implementation of a simple thread pool which
 *  assigns tasks to threads
 * @tparam TaskArgs The arguments for the task functor
 */
template<typename... TaskArgs>
class SimpleThreadPool : public ThreadPool
{
protected:
  using ThreadPool::io_context_;
  using ThreadPool::pool_size;
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Number of available threads, those which are not currently
   *  performing a task
   */
  std::size_t available_;
  /**
   * @brief Mutex to handle concurrent tasks
   */
  boost::mutex mutex_;
  /**
   * @brief Condition variable to handle tasks dispatching depending on
   *  available threads
   */
  boost::condition_variable cond_;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple thread pool constructor
   * @see ThreadPool::pool_size
   * @see ThreadPool::ThreadPool
   */
  explicit SimpleThreadPool(std::size_t const _pool_size)
    : ThreadPool(_pool_size)
    , available_(_pool_size)
  {
  }
  ~SimpleThreadPool() override = default;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Run a task when there is an available thread for it
   */
  template<typename Task>
  void run_task(Task task)
  {
    boost::unique_lock<boost::mutex> lock(mutex_);

    // If no threads are available, then wait for a thread to finish.
    if (0 == available_) {
      cond_.wait(lock);
    }

    // Decrement count, indicating thread is no longer available.
    --available_;

    // Unlock the mutex
    lock.unlock();

    // Post a wrapped task into the queue
    boost::asio::post(io_context_,
                      boost::bind(&SimpleThreadPool::wrap_task,
                                  this,
                                  boost::function<void(TaskArgs...)>(task)));
  }

  /**
   * @brief Lock until all pending threads have finished
   */
  virtual void join()
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    while (available_ < pool_size) {
      cond_.wait(lock);
    }
  }

protected:
  /**
   * @brief Wrap a task so that available threads count can be increased
   *  once provided task has been completed
   * @param task Task to be wrapped
   */
  virtual void wrap_task(boost::function<void(TaskArgs...)>& task)
  {
    // Run the user supplied task.
    try {
      do_task(task);
    }
    // Suppress all exceptions.
    catch (const std::exception& e) {
      std::stringstream ss;
      ss << "ThreadPool::wrap_task EXCEPTION: " << e.what();
      logging::WARN(ss.str());
    }

    // Task has finished, so increment count of available threads.
    boost::unique_lock<boost::mutex> lock(mutex_);
    ++available_;
    lock.unlock();
    cond_.notify_one();
  }

  /**
   * @brief Invoke task with corresponding arguments
   * @param task Task to be invoked
   */
  virtual void do_task(boost::function<void(TaskArgs...)>& task) = 0;

public:
  // ***  EXTERNAL HANDLING  *** //
  // *************************** //
  /**
   * @brief Notify the conditional variable so if there is a waiting thread
   *  it will wake up
   */
  virtual inline void notifyOne() { cond_.notify_one(); }
};
