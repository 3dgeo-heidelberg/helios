#pragma once

#include <ThreadPool.h>
#include <TaskWarehouse.h>

#include <memory>

using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @tparam Task The type of task handled by the warehouse thread pool
 * @brief Thread pool which starts thread so they are always waiting for new
 *  tasks to be posted to the warehouse to compute them.
 */
template <typename Task>
class WarehouseThreadPool : public ThreadPool{
protected:
using ThreadPool::io_service_;
using ThreadPool::pool_size;
    //  ***  ATTRIBUTES  *** //
    // ********************* //
    /**
     * @brief The task warehouse used to handle tasks
     */
    TaskWarehouse<Task> warehouse;
    /**
     * @brief True if thread pool is working, false if it is already finished.
     *
     * NOTICE that if working is true threads are guaranteed to be started and
     *  not finished. In consequence, threads might be computing tasks or
     *  waiting for new tasks. A warehouse thread pool with all threads waiting
     *  for tasks is considered as working. A warehouse thread pool is not
     *  working only if it has finished accepting tasks.
     */
    bool working;
    /**
     * @brief Count how many active workers there are. It is used for final
     *  join
     */
    int workersCount;
    /**
     * @brief Count how many workers with pending tasks there are. It is used
     *  for join (not final join). It can also be read as working count (not
     *  workers count) because it counts how many working threads there are.
     */
    int pendingCount;
    /**
     * @brief Mutex to handle concurrent access to workers count
     */
    boost::mutex mtx;
    /**
     * @brief Conditional variable to handle concurrent access to workers
     *  count
     */
    boost::condition_variable condvar;
    /**
     * @brief Mutex to handle join until warehouse is empty (not the final
     *  join)
     * @see WarehouseThreadPool::join
     * @see WarehouseThreadPool::finalJoin
     */
    boost::mutex joinMtx;
    /**
     * @brief Conditional variable to handle join until warehouse is empty
     *  (not the final join)
     * @see WarehouseThreadPool::join
     * @see WarehouseThreadPool::finalJoin
     */
    boost::condition_variable joinCondvar;


    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Warehouse thread pool constructor
     * @see ThreadPool::pool_size
     * @see ThreadPool::ThreadPool
     */
    explicit WarehouseThreadPool(
        std::size_t const _pool_size,
        std::size_t const maxTasks=256
    ) :
        ThreadPool(_pool_size),
        warehouse(maxTasks),
        working(true),
        workersCount(0),
        pendingCount(0)
    {}
    ~WarehouseThreadPool() override = default;

public:
    // ***  WAREHOUSE THREADPOOL  *** //
    // ****************************** //
    /**
     * @brief Expose the warehouse post method
     * @see TaskWarehouse::post(shared_ptr<Task>)
     */
    virtual inline bool post(shared_ptr<Task> task)
    {return warehouse.post(task);}
    /**
     * @brief Expose the warehouse post method
     * @see TaskWarehouse::post(vector<shared_ptr<Task>>&)
     */
    virtual inline bool post(vector<shared_ptr<Task>> &tasks)
    {return warehouse.post(tasks);}
    /**
     * @brief Expose the warehouse get method
     * @see TaskWarehouse::get
     */
    virtual inline shared_ptr<Task> get()
    {return warehouse.get();}
    /**
     * @brief Expose the warehouse notify method
     * @see TaskWarehouse::notify
     */
    virtual inline void notify()
    {warehouse.notify();}
    /**
     * @brief Expose the warehouse notify all method
     * @see TaskWarehouse::notifyAll
     */
    virtual inline void notifyAll()
    {warehouse.notifyAll();}

    /**
     * @brief Start the warehouse thread pool.
     *
     * A non started warehouse thread pool will not compute any posted task
     *  until it is started.
     * @see WarehouseThreadPool::_start
     */
    virtual void start(){
        // Start threads
        workersCount = pool_size;
        for(size_t tid = 0 ; tid < pool_size ; ++tid){
            io_service_.post(
                boost::bind(
                    &WarehouseThreadPool::_start,
                    this,
                    tid
                )
            );
        }
    }

    /**
     * @brief Lock until warehouse is empty
     */
    virtual void join(){
        boost::unique_lock<boost::mutex> lock(joinMtx);
        while(pendingCount>0){
            joinCondvar.wait(lock);
        }
    }

    /**
     * @brief Finish the warehouse thread pool in a proper way. It is, allowing
     *  all tasks to finish properly and emptying the warehouse
     * @see WarehouseThreadPool::finalJoin
     */
    virtual void finish(){
        warehouse.notifyAllUpdate(working, false);
        finalJoin();
    }

protected:
    /**
     * @brief Start a thread.
     *
     * Started threads compute tasks from warehouse. If there are not remaining
     *  tasks they wait for a notify signal to resume its activity, to avoid
     *  active listening. To stop threads the WarehouseThreadPool::finish
     *  method must be invoked.
     *
     * @param tid Index for thread to be started
     * @see WarehouseThreadPool::warehouse
     * @see WarehouseThreadPool::finish
     * @see WarehouseThreadPool::start
     */
    virtual void _start(size_t const tid){
        shared_ptr<Task> task;

        // Standard working mode : Compute tasks and wait for new ones
        while(working){
            boost::unique_lock<boost::mutex> lockIncrease(joinMtx);
            ++pendingCount;
            lockIncrease.unlock();
            while( (task=warehouse.get()) != nullptr){
                doTask(tid, task);
            }
            boost::unique_lock<boost::mutex> lockDecrease(joinMtx);
            --pendingCount;
            lockDecrease.unlock();
            joinCondvar.notify_one();
            warehouse.waitIf(working);
        }

        // Finishing mode : Consume pending tasks
        if(warehouse.hasPendingTasks()){
            boost::unique_lock<boost::mutex> lockIncrease(joinMtx);
            ++pendingCount;
            lockIncrease.unlock();
            while( (task=warehouse.get()) != nullptr){
                doTask(tid, task);
            }
            boost::unique_lock<boost::mutex> lockDecrease(joinMtx);
            --pendingCount;
            lockDecrease.unlock();
            joinCondvar.notify_one();
        }

        // Finish : Decrement from workers count
        boost::unique_lock<boost::mutex> lock(mtx);
        --workersCount;
        lock.unlock();
        condvar.notify_one();
    }

    /**
     * @brief Thread execute given task. By default it assumes task must be
     *  called with no arguments. To handle tasks with arguments extend the
     *  WarehouseThreadPool and override this method accordingly.
     * @param tid Index/identifier of thread that must compute the task
     * @param task Task to be computed
     */
    virtual void doTask(size_t const tid, shared_ptr<Task> task){
        (*task)();
    }

    /**
     * @brief Lock until all pending tasks have been finished. If it is not
     *  called after finish, it will lead to an eternal waiting.
     * @see WarehouseThreadPool::finish
     */
    virtual void finalJoin(){
        boost::unique_lock<boost::mutex> lock(mtx);
        while(workersCount > 0){
            condvar.wait(lock);
        }
    }


};