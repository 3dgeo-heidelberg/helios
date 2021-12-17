#pragma once

#include <ThreadPool.h>

/**
 * @version 1.0
 * @brief Abstract class providing implementation of a simple thread pool which
 *  assigns tasks to threads
 * @tparam TaskArgs The arguments for the task functor
 */
template <typename ... TaskArgs>
class SimpleThreadPool : public ThreadPool{
protected:
using ThreadPool::io_service_;
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
    explicit SimpleThreadPool(std::size_t const _pool_size) :
        ThreadPool(_pool_size),
        available_(_pool_size)
    {}
    virtual ~SimpleThreadPool() = default;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
	 * @brief Run a task when there is an available thread for it
	 */
    template <typename Task>
    void run_task(Task task){
        boost::unique_lock<boost::mutex> lock(mutex_);

        // If no threads are available, then wait for a thread to finish.
        if (0 == available_){
            cond_.wait(lock);
        }

        // Decrement count, indicating thread is no longer available.
        --available_;

        // Unlock the mutex
        lock.unlock();

        // Post a wrapped task into the queue
        io_service_.post(
            boost::bind(
                &SimpleThreadPool::wrap_task,
                this,
                boost::function<void(TaskArgs ...)>(task)
            )
        );
    }

    /**
     * @brief Lock until all pending threads have finished
     */
    virtual void join(){
        boost::unique_lock<boost::mutex> lock(mutex_);
        while(available_ < pool_size){
            cond_.wait(lock);
        }
    }

protected:
    /**
	 * @brief Wrap a task so that available threads count can be increased
	 *  once provided task has been completed
	 * @param task Task to be wrapped
	 */
    virtual void wrap_task(
        boost::function<void(TaskArgs ...)> &task
    ){
        // Run the user supplied task.
        try{
            do_task(task);
        }
            // Suppress all exceptions.
        catch (const std::exception &e) {
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
    virtual void do_task(
        boost::function<void(TaskArgs ...)> &task
        ) = 0;

public:
    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the number of available threads
     * @return Number of available threads
     * @see SimpleThreadPool::available_
     * @see SimpleThreadPool::setAvailable
     */
    virtual inline size_t getAvailable() {return available_;}
    /**
     * @brief Set the number of available threads
     *
     * Use this setter with caution and only if you know what you are doing.
     *
     * Inproper modification of available threads might ruin simple thread
     *  pool behavior leading to eternal waiting or other unexpected (and
     *  probably unwanted) scenarios.
     *
     * @param available New number of available threads
     * @see SimpleThreadPool::available_
     * @see SimpleThreadPool::getAvailable
     */
    virtual inline void setAvailable(size_t const available)
    {available_ = available;}

};
