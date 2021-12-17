#pragma once

#include <SharedSubTaskCompletionHandler.h>

#include <boost/thread.hpp>

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief A shared task is said to be a collection of shared sub-tasks. Each
 *  shared sub-task can be computed in parallel with respect to its siblings.
 * @see SharedTaskSequencer
 */
class SharedSubTask{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The shared sub-task completion handler that handles what must be
     *  done after a shared sub-task execution has been finished. The most
     *  paradigmatic case of a valid task completion handler is the
     *  shared task sequencer.
     * @see SharedTaskSequencer
     * @see SharedSubTaskCompletionHandler
     */
    std::shared_ptr<SharedSubTaskCompletionHandler> ch;
    /**
     * @brief The key identifying the shared sub task inside the shared task
     *  sequencer context
     * @see SharedTaskSequencer
     * @see SharedTaskSequencer::subTasks
     */
    size_t key = 0;
    /**
     * @brief The thread associated with the shared sub-tasks. It is nullptr
     *  until the shared sub-task has been started by its corresponding
     *  shared task sequencer
     */
    std::shared_ptr<boost::thread> thread = nullptr;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for shared sub-task
     * @see SharedSubTask::tch
     */
    SharedSubTask(std::shared_ptr<SharedSubTaskCompletionHandler> ch) :
        ch(ch),
        key(0)
    {}
    virtual ~SharedSubTask() = default;

    // ***  FUNCTOR  *** //
    // ***************** //
    /**
     * @brief The functor that will be called by any thread. It calls the
     *  SharedSubTask::run method to solve/compute the sub-task. Also, once the
     *  task has been computed, it delegates upon the task completion handler.
     * @see SharedSubTask::ch
     */
    virtual inline void operator() (){
        run();
        ch->onSharedSubTaskCompletion(key);
    }

    // ***  RUNNABLE SHARED TASK  *** //
    // ****************************** //
    /**
     * @brief Abstract run function that must be implemented by any derived
     *  class which pretends to provide a concrete implementation of a
     *  SharedSubTask
     */
    virtual void run() = 0;
    /**
     * @brief Post-processing to be applied after shared sub-task has
     *  finished. By default it is a void function which does nothing, but it
     *  can be overridden.
     */
    virtual void postProcess() {}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the key of the shared sub-task inside the shared task
     *  sequencer context
     * @return Key of the shared sub-task inside the shared task sequencer
     *  context
     * @see SharedSubTask::key
     */
    virtual size_t getKey() {return key;}
    /**
     * @brief Set the key of the shared sub-task inside the shared task
     *  sequencer context
     * @param key Key of the shared sub-task inside the shared task
     *  sequencer context
     * @see SharedSubTask::key
     */
    virtual void setKey(size_t const key) {this->key = key;}
    /**
     * @brief Get the thread associated to the shared sub-task
     * @return Thread associated to the shared sub-task
     * @see SharedSubTask::thread
     */
    virtual std::shared_ptr<boost::thread> getThread() {return thread;}
    /**
     * @brief Set the thread associated to the shared sub-task
     * @param thread Thread associated to the shared sub-task
     * @see SharedSubTask::thread
     */
    virtual void setThread(std::shared_ptr<boost::thread> thread)
    {this->thread = thread;}
};