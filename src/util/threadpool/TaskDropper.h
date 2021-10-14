#pragma once

#include <util/HeliosException.h>
#include <util/threadpool/ThreadPool.h>

#include <vector>
#include <memory>

using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @tparam TaskType Type of task handled by the task dropper
 * @tparam TaskArgs Arguments for the functor of the task handled by the
 *  task dropper
 *
 * @brief Class which handles tasks dropping. It is, executing and then
 *  removing each task when dropping.
 */
template <typename TaskType, typename ThreadPoolType, typename ... TaskArgs>
class TaskDropper {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Tasks to be dropped as a whole.
     *
     * Task vector is denoted in the documentation as \f$\vec{T}\f$
     */
    std::vector<shared_ptr<TaskType>> tasks;
    /**
     * @brief Specify the maximum number of tasks before forcing a drop.
     *
     * If it is \f$m=0\f$, then forcing a drop will never happen.
     * If it is \f$m>0\f$, then forcing a drop will happen as soon as
     *  \f$\vert\vec{T}\vert = m\f$ is satisfied
     */
    size_t maxTasks;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for TaskDropper
     * @param maxTasks Value to initialize maximum tasks limit
     * @see TaskDropper::maxTasks
     */
    TaskDropper(size_t maxTasks=32) : maxTasks(maxTasks) {}
    virtual ~TaskDropper() = default;


    // ***  TASK DROPPER METHODS *** //
    // ***************************** //
    /**
     * @brief Add a task to the TaskDropper
     * @param task Task to be added
     * @return True if dropper has dropped its tasks, false otherwise
     */
    virtual inline bool add(shared_ptr<TaskType> task){
        tasks.push_back(task);
        if(tasks.size() == maxTasks){
            drop();
            return true;
        }
        return false;
    }
    /**
     * @brief Like TaskDropper::add but for tasks with arguments
     * @param args Arguments as common input for each task. They will be used
     *  just in case maxTasks limit has been reached
     * @return True if dropper has dropped its tasks, false otherwise
     * @see TaskDropper::add
     */
    virtual inline bool add(shared_ptr<TaskType> task, TaskArgs ... args){
        tasks.push_back(task);
        if(tasks.size() == maxTasks){
            drop(args...);
            return true;
        }
        return false;
    }
    /**
     * @brief Like TaskDropper::add but running the drop method in parallel
     *  through a callback from given thread pool
     * @param pool Thread pool to be used to do the the drop method callback
     *  in parallel
     * @return True if dropper has dropped its tasks, false otherwise
     * @see TaskDropper::add
     * @see TaskDropper::drop(ThreadPoolType &)
     * @see ThreadPool
     */
    virtual inline bool add(ThreadPoolType &pool, shared_ptr<TaskType> task){
        tasks.push_back(task);
        if(tasks.size() == maxTasks){
            drop(pool);
            return true;
        }
        return false;
    }

    /**
     * @brief Drop all tasks, one after another.
     *
     * Dropped tasks will be executed and then released from tasks vector.
     * Notwithstanding, at least for this default implementation, tasks will
     *  not be removed from tasks vector until all have been executed
     */
    virtual inline void drop(){
        for(shared_ptr<TaskType> task : tasks) doTask(*task);
        tasks.clear();
    }
    /**
     * @brief Drop all tasks with arguments, one after another.
     *
     * This method works like TaskDropper::drop but for tasks with arguments
     *
     * @param args Arguments for the task to be executed
     * @see TaskDropper::drop
     */
    virtual inline void drop(TaskArgs ... args){
        for(shared_ptr<TaskType> task : tasks) doTask(*task, args...);
        tasks.clear();
    }
    /**
     * @brief A callback for TaskDropper::drop but using a thread pool to be
     *  executed in parallel.
     *
     * The callback TaskDropper::drop will be called through functor operator,
     *  with or without arguments depending on the thread pool implementation
     *
     * @param pool Thread pool to be used for parallel execution
     * @see TaskDropper::drop
     * @see TaskDropper::add(ThreadPoolType &, shared_ptr<TaskType>)
     * @see ThreadPool
     */
    virtual inline void drop(ThreadPoolType &pool){
        throw HeliosException(
            "TaskDropper::drop(ThreadPoolType &, TaskType &) cannot be used.\n"
            "It must be overridden by any derived class providing parallel "
            "execution through given thread pool."
        );
    };

protected:
    // ***  INTERNAL TASK DROPPER METHODS  *** //
    // *************************************** //
    /**
     * @brief Execute a task with no arguments.
     *
     * For implementations handling tasks that mandatory require arguments it
     *  is recommended to override this method to throw an exception
     *
     * @param task Task to be executed
     */
    virtual inline void doTask(TaskType &task)
    {task();}
    /**
     * @brief Execute a task with corresponding arguments.
     *
     * @param task Task to be executed
     * @param args Arguments for the task to be executed
     */
    virtual inline void doTask(TaskType &task, TaskArgs ... args)
    {task(args...);}

public:
    // ***  FUNCTOR OPERATORS  *** //
    // *************************** //
    /**
     * @brief Void functor operator calling drop method to be compatible with
     *  thread pools
     * @see TaskDropper::drop
     */
    virtual void operator()(){drop();}
    /**
     * @brief Functor operator with arguments calling drop method with
     *  arguments which can be used by thread pools
     * @param args Arguments for tasks to be executed (the same arguments
     *  for each task)
     * @see TaskDRopper::drop
     */
    virtual void operator()(TaskArgs ... args){drop(args...);}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Get current maximum tasks limit
     * @return Current maximum tasks limit
     * @see TaskDropper::maxTasks
     */
    virtual inline size_t getMaxTasks() const
    {return maxTasks;}
    /**
     * @brief Set new maximum tasks limit
     * @param maxTasks New maximum tasks limit
     * @see TaskDropper::maxTasks
     */
    virtual inline void setMaxTasks(size_t const maxTasks)
    {this->maxTasks = maxTasks;}

};