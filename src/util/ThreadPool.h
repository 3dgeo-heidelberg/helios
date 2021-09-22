#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <logging.hpp>
#include <sstream>

/**
 * @version 1.0
 * @brief Class providing core implementation of a thread pool to deal with
 *  multi threading tasks
 */
template <typename ... TaskArgs>
class ThreadPool{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Instance of boost input/output service for asynchronous data
     *  processing
     */
	boost::asio::io_service io_service_;
	/**
	 * @brief Instance of work to report the io service when it has pending
	 *  tasks
	 * @see ThreadPool::io_service_
	 */
	boost::asio::io_service::work work_;
	/**
	 * @brief Size of thread pool (number of threads)
	 */
    std::size_t pool_size;
    /**
     * @brief Group of threads
     */
	boost::thread_group threads_;
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
     * @brief Condition var to handle tasks dispatching depending on
     *  available threads
     */
	boost::condition_variable cond_;

    /**
     * @brief Array of flags specifying availability of resource sets
     *
     * The ith element of this array correspond to the ith resource for any
     *  resource set/array
     */
    bool *resourceSetAvailable;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Thread pool constructor
     * @see ThreadPool::pool_size
     */
    explicit ThreadPool(std::size_t const _pool_size) :
        work_(io_service_),
        pool_size(_pool_size),
        available_(_pool_size)
    {
        // Allocate
        resourceSetAvailable = new bool[pool_size];

        // Initialize
        for (std::size_t i = 0; i < pool_size; ++i){
            resourceSetAvailable[i] = true;
            threads_.create_thread(
                [&] () -> boost::asio::io_context::count_type{
                    return io_service_.run();
                }
            );
        }
    }

    virtual ~ThreadPool(){
        // Force all threads to return from io_service::run().
        io_service_.stop();

        // Suppress all exceptions.
        try{
            threads_.join_all();
        }
        catch (const std::exception&) {}

        delete[] resourceSetAvailable;
    }

protected:
    // ***  M E T H O D S  *** //
    // *********************** //
	/**
	 * @brief Obtain the index of an available resource set
	 * @return Available resource set index
	 */
	virtual inline int getAvailableResourceSetIndex() const {
	    for(size_t i = 0 ; i  < pool_size ; i++){
	        if(resourceSetAvailable[i]) return i;
	    }
	    return -1;
	}


public:
    /**
     * @brief Obtain the thread pool size
     * @return Thread pool size
     * @see ThreadPool::pool_size
     */
    virtual inline std::size_t getPoolSize() const {return pool_size;}


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

        // Get resource set index
        int const resourceIdx = getAvailableResourceSetIndex();
        resourceSetAvailable[resourceIdx] = false;

        // Unlock the mutex
        lock.unlock();

        // Post a wrapped task into the queue
        io_service_.post(
            boost::bind(
                &ThreadPool<TaskArgs ...>::wrap_task,
                this,
                boost::function<void(TaskArgs ...)>(task),
                resourceIdx
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
	 * @param resourceIdx Resource index associated with the task. It is
	 *  necessary to release associated resources so other tasks can use
	 *  them later
	 */
	virtual void wrap_task(
	    boost::function<void(TaskArgs ...)> &task,
	    int const resourceIdx
    ){
		// Run the user supplied task.
		try{
			do_task(task, resourceIdx);
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
		resourceSetAvailable[resourceIdx] = true;
		cond_.notify_one();
	}

	/**
	 * @brief Invoke task with corresponding arguments
	 * @param task Task to be invoked
	 * @param resourceIdx Index of resources associated with thread invoking
	 *  the task
	 */
	virtual void do_task(
        boost::function<void(TaskArgs ...)> &task,
        int const resourceIdx
    ) = 0;
};
