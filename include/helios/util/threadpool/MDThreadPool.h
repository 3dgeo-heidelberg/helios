#pragma once

#include <boost/asio/post.hpp>
#include <helios/util/HeliosException.h>
#include <helios/util/threadpool/SimpleThreadPool.h>
#include <unordered_map>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class extending basic thread pool implementation to provide
 *  a basis layer to handle tasks with associated data.
 *
 * NOTICE available variable from ThreadPool has a different usage in
 *  MDThreadPool. Instead of counting available threads to deal with tasks,
 *  it counts pending tasks to be finished
 *
 * @see ThreadPool
 */
template<typename MDType, typename... TaskArgs>
class MDThreadPool : public SimpleThreadPool<TaskArgs...>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Multiple data thread pool constructor
   * @see ThreadPool::ThreadPool(std::size_t const)
   */
  explicit MDThreadPool(std::size_t const _pool_size)
    : SimpleThreadPool<TaskArgs...>(_pool_size)
  {
    setPendingTasks(0);
  }
  virtual ~MDThreadPool() {}

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Run a task with associated data without considering limit for
   *  max pending tasks
   */
  template<typename Task>
  void run_md_task(Task task, MDType* data)
  {
    // Lock run task mutex
    boost::unique_lock<boost::mutex> lock(this->mutex_);

    // Increment number of pending tasks
    increasePendingTasks();

    // Unlock run task mutex
    lock.unlock();

    // Post a wrapped task into the queue
    boost::asio::post(
      this->io_context_,
      boost::bind(&MDThreadPool<MDType, TaskArgs...>::wrap_md_task,
                  this,
                  boost::function<void(TaskArgs...)>(task),
                  data));
  }

  /**
   * @brief Try to run a task with associated data. If the max limit of
   *  pending tasks has not been reached, the task will be run asynchronously
   *  and true will be returned once it has been posted to the asynchronous
   *  execution service. If the max limit has been reached, then the task
   *  will not be posted to the asynchronous execution service and false
   *  will be returned.
   * @return True if task was posted, false otherwise
   */
  template<typename Task>
  bool try_run_md_task(Task task, MDType* data)
  {
    // Lock run task mutex
    boost::unique_lock<boost::mutex> lock(this->mutex_);

    // Check pending tasks does not exceed max limit
    if (getPendingTasks() >= this->pool_size) {
      lock.unlock();
      return false;
    }

    // Increment number of pending tasks
    increasePendingTasks();

    // Unlock run task mutex
    lock.unlock();

    // Post a wrapped task into the queue
    boost::asio::post(
      this->io_context_,
      boost::bind(&MDThreadPool<MDType, TaskArgs...>::wrap_md_task,
                  this,
                  boost::function<void(TaskArgs...)>(task),
                  data));
    return true;
  }

  /**
   * @brief Lock until all pending threads have finished
   */
  void join() override
  {
    boost::unique_lock<boost::mutex> lock(this->mutex_);
    while (getPendingTasks() > 0) {
      this->cond_.wait(lock);
    }
    this->work_guard_.reset(); // Allow works to finish normally
    this->threads_.join_all(); // Wait for all threads to finish
  }

protected:
  /**
   * @brief Wrap a data task so that pending tasks count can be increased
   *  once provided data task has been completed
   * @param task Data task to be wrapped
   * @param data Pointer to data associated with the task. It is necessary
   *  to handle task arguments. If data must be released after computation,
   *  do it at do_md_task implementation. It is safe to do so because it will
   *  not be used later by any MDThreadPool stage
   */
  virtual void wrap_md_task(boost::function<void(TaskArgs...)>& task,
                            MDType* data)
  {
    // Run supplied data task
    try {
      do_md_task(task, data);
    }
    // Suppress all exceptions.
    catch (const std::exception& e) {
      std::stringstream ss;
      ss << "MDThreadPool::wrap_md_task EXCEPTION: " << e.what();
      logging::WARN(ss.str());
    }

    // Task has finished, so increment count of available threads.
    boost::unique_lock<boost::mutex> lock(this->mutex_);
    decreasePendingTasks();
    if (getPendingTasks() == 0) {
      lock.unlock();
      this->cond_.notify_one();
    }
  }

  /**
   * @brief Throw exception when calling non data do_task
   * @see ThreadPool::do_task
   */
  void do_task(boost::function<void(TaskArgs...)>& task) override
  {
    throw HeliosException("MDThreadPool::do_task MUST NOT be invoked.\n"
                          "Please, avoid this call or override implementation");
  }

  /**
   * @brief Invoke task with associated data
   * @param task Task to be invoked
   * @param data The data associated with the task
   */
  virtual void do_md_task(boost::function<void(TaskArgs...)>& task,
                          MDType* data) = 0;

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the number of pending tasks to be computed
   * @return Number of pending tasks to be computed
   */
  virtual inline size_t getPendingTasks() { return this->available_; }
  /**
   * @brief Obtain the number of pending tasks to be computed in a thread
   *  safe way. It is, with proper handling of concurrent reading.
   * @return Number of pending tasks to be computed
   */
  virtual inline size_t safeGetPendingTasks()
  {
    boost::unique_lock<boost::mutex> lock(this->mutex_);
    size_t const pTasks = getPendingTasks();
    lock.unlock();
    return pTasks;
  }
  /**
   * @brief Set the number of pending tasks to be computed
   * @param pendingTasks New number of pendings tasks to be computed
   */
  virtual inline void setPendingTasks(size_t const pendingTasks)
  {
    this->available_ = pendingTasks;
  }
  /**
   * @brief Set the number of pending tasks to be computed in a thread safe
   *  way. It is, with proper handling of current writting.
   * @param pendingTasks New number of pending tasks to be computed
   */
  virtual inline void safeSetPendingTasks(size_t const pendingTasks)
  {
    boost::unique_lock<boost::mutex> lock(this->mutex_);
    setPendingTasks(pendingTasks);
    lock.unlock();
  }
  /**
   * @brief Subtract specified amount of pending tasks
   * \f[
   *  p_{t+1} = p_{t} - n
   * \f]
   * @param amount Amount of pending tasks to be subtracted
   */
  virtual inline void subtractPendingTasks(size_t const amount)
  {
    setPendingTasks(getPendingTasks() - amount);
  }
  /**
   * @brief Subtract pending tasks in a thread safe way. It is, with proper
   *  handling of concurrency
   * @see MDThreadPool::subtractPendingTasks
   */
  virtual inline void safeSubtractPendingTasks(size_t const amount)
  {
    boost::unique_lock<boost::mutex> lock(this->mutex_);
    subtractPendingTasks(amount);
    lock.unlock();
  }
  /**
   * @brief Unitary increase the number of pending tasks to be computed
   */
  virtual inline void increasePendingTasks() { ++(this->available_); }
  /**
   * @brief Unitary decrease the number of pending tasks to be computed
   */
  virtual inline void decreasePendingTasks() { --(this->available_); }
};
