#pragma once

#include <ThreadPool.h>
#include <HeliosException.h>

/**
 * @verison 1.0
 * @brief Absract class extending basic thread pool implementation to provided
 *  a basis layer to handle thread associated resources
 * @see ThreadPool
 */
template <typename ... TaskArgs>
class ResThreadPool : public ThreadPool<TaskArgs ...>{
protected:
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
     * @brief Resource thread pool constructor
     * @see ThreadPool::ThreadPool(size_t const)
     */
    explicit ResThreadPool(std::size_t const _pool_size) :
        ThreadPool<TaskArgs ...>(_pool_size)
    {
        // Allocate
        resourceSetAvailable = new bool[this->pool_size];

        // Initialize
        for (std::size_t i = 0; i < this->pool_size; ++i){
            resourceSetAvailable[i] = true;
        }
    }

    virtual ~ResThreadPool(){
        // Release memory
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
        for(size_t i = 0 ; i  < this->pool_size ; i++){
            if(resourceSetAvailable[i]) return i;
        }
        return -1;
    }

public:
    /**
	 * @brief Run a task with associated resources when there is an available
     *  thread for it
	 */
    template <typename Task>
    void run_res_task(Task task){
        boost::unique_lock<boost::mutex> lock(this->mutex_);

        // If no threads are available, then wait for a thread to finish.
        if (0 == this->available_){
            this->cond_.wait(lock);
        }

        // Decrement count, indicating thread is no longer available.
        --(this->available_);

        // Get resource set index
        int const resourceIdx = getAvailableResourceSetIndex();
        resourceSetAvailable[resourceIdx] = false;

        // Unlock the mutex
        lock.unlock();

        // Post a wrapped task into the queue
        this->io_service_.post(
            boost::bind(
                &ResThreadPool<TaskArgs ...>::wrap_res_task,
                this,
                boost::function<void(TaskArgs ...)>(task),
                resourceIdx
            )
        );
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
    virtual void wrap_res_task(
        boost::function<void(TaskArgs ...)> &task,
        int const resourceIdx
    ){
        // Run the user supplied task.
        try{
            do_res_task(task, resourceIdx);
        }
            // Suppress all exceptions.
        catch (const std::exception &e) {
            std::stringstream ss;
            ss << "ResThreadPool::wrap_res_task EXCEPTION: " << e.what();
            logging::WARN(ss.str());
        }

        // Task has finished, so increment count of available threads.
        boost::unique_lock<boost::mutex> lock(this->mutex_);
        ++(this->available_);
        resourceSetAvailable[resourceIdx] = true;
        this->cond_.notify_one();
    }

    /**
     * @brief Throw exception when calling non resource do_task
     * @see ThreadPool::do_task
     */
    void do_task(
        boost::function<void(TaskArgs ...)> &task
    ) override {
        throw HeliosException(
            "ResThreadPool::do_task MUST NOT be invoked.\n"
            "Please, avoid this call or override implementation"

        );
    }

    /**
	 * @brief Invoke task with associated resources with corresponding
     *  arguments
	 * @param task Task to be invoked
	 * @param resourceIdx Index of resources associated with thread invoking
	 *  the task
	 */
    virtual void do_res_task(
        boost::function<void(TaskArgs ...)> &task,
        int const resourceIdx
    ) = 0;

};