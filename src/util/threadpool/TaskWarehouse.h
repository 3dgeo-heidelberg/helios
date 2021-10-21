#pragma once

#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/shared_lock_guard.hpp>

#include <vector>
#include <memory>


using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a warehouse to store and retrieve tasks in a
 *  thread safe fashion
 * @tparam Task Template Type of tasks handled by the task warehouse
 */
template <typename Task>
class TaskWarehouse{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Maximum number of tasks the warehouse can handle
     */
    size_t maxTasks;
    /**
     * @brief Tasks handled by the warehouse
     */
    vector<shared_ptr<Task>> tasks;
    /**
     * @brief The mutex to handle concurrent access to the tasks
     */
    boost::shared_mutex mtx;
    /**
     * @brief The wait mutex to implement non-active listening waiting
     */
    boost::shared_mutex wmtx;
    /**
     * @brief Conditional variable to handle wait/notify signals
     */
    boost::condition_variable_any condvar;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for task warehouse
     */
    TaskWarehouse(size_t const maxTasks=256) : maxTasks(maxTasks) {}
    virtual ~TaskWarehouse() = default;

    // ***  WAREHOUSE METHODS  *** //
    // *************************** //
    /**
     * @brief Post task into warehouse. Posted task will be stored in the
     *  warehouse only if there is enough available space.
     * @param task Task to be posted
     * @return True if task was successfully stored in warehouse, false
     *  otherwise
     */
    virtual bool post(shared_ptr<Task> task){
        boost::unique_lock<boost::shared_mutex> writeLock(mtx);
        if(tasks.size() >= maxTasks) return false;
        tasks.push_back(task);
        return true;
    }
    /**
     * @brief Post tasks into warehouse. Posted tasks will be stored in the
     *  warehouse only if there is enough available space for all of them.
     * It is, if \f$n\f$ tasks are posted but there is only enough space to
     *  store \f$n-1\f$, then not even a single task will be stored.
     * @param _tasks Tasks to be posted
     * @return True if tasks were successfully stored in warehouse, false
     *  otherwise
     */
    virtual bool post(vector<shared_ptr<Task>> &_tasks){
        boost::unique_lock<boost::shared_mutex> writeLock(mtx);
        if( (tasks.size()+_tasks.size()) > maxTasks) return false;
        tasks.insert(tasks.end(), _tasks.begin(), _tasks.end());
        return true;
    }

    /**
     * @brief Get task from warehouse, if any. Retrieved task is removed from
     *  warehouse
     * @return Retrieved task
     */
    virtual shared_ptr<Task> get(){
        boost::unique_lock<boost::shared_mutex> writeLock(mtx);
        if(tasks.empty()) return nullptr;
        shared_ptr<Task> task = tasks.back();
        tasks.pop_back();
        return task;
    }
    /**
     * @brief Get tasks from warehouse, if any
     * @param n How many tasks retrieve
     * @return Retrieved tasks. If there are no tasks to retrieve, then
     *  empty vector is returned. If there are \f$<n\f$ tasks, then as many
     *  as possible are returned
     */
    virtual vector<shared_ptr<Task>> get(size_t const n){
        vector<shared_ptr<Task>> _tasks;
        typename vector<shared_ptr<Task>>::iterator begin, end;
        boost::unique_lock<boost::shared_mutex> writeLock(mtx);
        if(tasks.empty()) return _tasks;
        begin = tasks.end()-n;
        end = tasks.end();
        _tasks.insert(_tasks.end(), begin, end);
        tasks.erase(begin, end);
        return _tasks;
    }

    /**
     * @brief Caller thread waits until notified
     */
    virtual inline void wait(){
        boost::shared_lock<boost::shared_mutex> lock(wmtx);
        condvar.wait(lock);
    }
    /**
     * @brief Caller thread waits until notified but only if condition is
     *  satisfied (true)
     * @param cond Wait condition
     */
    virtual inline void waitIf(bool const &cond){
        boost::shared_lock<boost::shared_mutex> lock(wmtx);
        if(cond) condvar.wait(lock);
    }
    /**
     * @brief Notify a waiting thread to stop waiting
     */
    virtual inline void notify(){condvar.notify_one();}
    /**
     * @brief Notify all waiting threads to stop waiting
     */
    virtual inline void notifyAll(){condvar.notify_all();}
    /**
     * @brief Notify a waiting thread after updating flag with new value in
     *  a thread-safe way
     * @param flag Flag to be updated
     * @param newValue New value for the flag
     */
    virtual inline void notifyUpdate(bool &flag, bool const newValue){
        boost::unique_lock<boost::shared_mutex> lock(wmtx);
        flag = newValue;
        condvar.notify_one();
    }
    /**
     * @brief Notify all waiting threads after updating flag with new value in
     *  a thread-safe way
     * @param flag Flag to be updated
     * @param newValue New value for the flag
     */
    virtual inline void notifyAllUpdate(bool &flag, bool const newValue){
        boost::unique_lock<boost::shared_mutex> lock(wmtx);
        flag = newValue;
        condvar.notify_all();
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Check if there are pending tasks in the warehouse (true) or not
     *  (false)
     * @return True if there are pending tasks, false otherwise
     */
    bool hasPendingTasks() {
        boost::shared_lock<boost::shared_mutex> lock(mtx);
        return !tasks.empty();
    }
};