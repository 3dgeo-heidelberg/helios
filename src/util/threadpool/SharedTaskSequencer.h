#pragma once

#include <SharedSubTaskCompletionHandler.h>
#include <SharedSubTask.h>

#include <boost/thread.hpp>

#include <memory>
#include <unordered_map>

using std::size_t;
using std::unordered_map;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class providing core implementation of a shared task sequencer. It is
 *  a sequencer which launches shared sub-tasks belonging to the same shared
 *  task.
 * @see SharedSubTask
 * @see SharedSubTaskCompletionHandler
 */
class SharedTaskSequencer : public SharedSubTaskCompletionHandler{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Maximum number of supported concurrent threads.
     *
     * If it is 0, it means that there is no limit to concurrent threads.
     */
    size_t maxThreads = 0;
    /**
     * @brief How many threads there are available to compute a task
     */
    size_t availableThreads = 0;
    /**
     * @brief Key of next shared sub-task to be inserted
     */
    size_t nextSharedSubTaskKey = 0;
    /**
     * @brief The collection of running shared sub-tasks.
     *
     * Only queued and running shared sub-tasks are included. Those which are
     *  fully finished or were not started yet will not be here.
     */
    unordered_map<size_t, std::shared_ptr<SharedSubTask>> subTasks;
    /**
     * @brief Mutex to handle concurrent start of sub-tasks
     */
    boost::mutex mtx;
    /**
     * @brief Condition variable to handle concurrent start of sub-tasks
     */
    boost::condition_variable condvar;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for shared task sequencer
     * @see SharedTaskSequencer::maxThreads
     */
    explicit SharedTaskSequencer(size_t const maxThreads) :
        maxThreads(maxThreads),
        availableThreads(maxThreads),
        nextSharedSubTaskKey(0)
    {}
    ~SharedTaskSequencer() override = default;

    // ***  SHARED TASK HANDLING  *** //
    // ****************************** //
    /**
     * @brief Start given shared sub-task
     * @param subTask Shared sub-task to be started
     * @see SharedSubTask
     * @see SharedTaskSequencer::startThread
     */
    virtual void start(std::shared_ptr<SharedSubTask> subTask);
    /**
     * @brief Assist the start method to start a shared sub-task by wrapping
     *  it into a SmartSharedFunctorContainer
     * @param subTask The sub-task to be started
     * @see SharedTaskSequencer::start
     * @see SharedSubTask
     * @see SmartSharedFunctorContainer
     */
    virtual void startThread(std::shared_ptr<SharedSubTask> subTask);
    /**
     * @brief Handle completion of each shared sub-task
     * @see SharedSubTaskCompletionHandler::onSharedSubTaskCompletion
     */
    void onSharedSubTaskCompletion(size_t const key) override;
    /**
     * @brief Wait until all pending sub-tasks have been finished
     */
    virtual void joinAll();

};