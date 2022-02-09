#include <SharedTaskSequencer.h>
#include <SmartSharedFunctorContainer.h>

// ***  SHARED TASK HANDLING  *** //
// ****************************** //
void SharedTaskSequencer::start(std::shared_ptr<SharedSubTask> subTask){
    // Handle case when there is no need to limit number of threads
    if(maxThreads == 0){
        boost::unique_lock<boost::mutex> uniqueLock(mtx);
        startThread(subTask);
        uniqueLock.unlock();
        return;
    }

    // Handle case when it is necessary to limit number of threads
    boost::unique_lock<boost::mutex> uniqueLock(mtx);
    while(availableThreads < 1){ // Wait for available threads
        condvar.wait(uniqueLock);
    }
    --availableThreads;
    startThread(subTask);
    uniqueLock.unlock();
}

void SharedTaskSequencer::startThread(std::shared_ptr<SharedSubTask> subTask){
    subTask->setKey(nextSharedSubTaskKey);
    subTasks.emplace(nextSharedSubTaskKey, subTask);
    subTask->setThread(std::make_shared<boost::thread>(
        SmartSharedFunctorContainer<SharedSubTask>(subTask)
    ));
    ++nextSharedSubTaskKey;
}

void SharedTaskSequencer::onSharedSubTaskCompletion(size_t const key){
    boost::unique_lock<boost::mutex> uniqueLock(mtx);
    if(maxThreads > 0) ++availableThreads;
    std::shared_ptr<SharedSubTask> subTask = subTasks[key];
    subTasks.erase(key);
    uniqueLock.unlock();
    condvar.notify_one();
    subTask->postProcess();
}

void SharedTaskSequencer::joinAll(){
    boost::unique_lock<boost::mutex> uniqueLock(mtx);
    while(subTasks.size() > 0){
        condvar.wait(uniqueLock);
    }
}
