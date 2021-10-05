#include <MultiThreadSAHKDTreeFactory.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
MultiThreadSAHKDTreeFactory::MultiThreadSAHKDTreeFactory(
    shared_ptr<SimpleKDTreeFactory> const kdtf,
    size_t const numJobs
) : MultiThreadKDTreeFactory(kdtf, numJobs)
{
    // sahkdtf by casting kdtf
    sahkdtf = std::static_pointer_cast<SAHKDTreeFactory>(kdtf);

    // Replace sahkdtf ILOT lock/unlock functions
    sahkdtf->_lockILOT = [&] () -> void {
        this->ilotCacheLock.reset(
            new boost::unique_lock<boost::mutex>(this->ilotCacheMutex)
        );
    };
    sahkdtf->_unlockILOT = [&] () -> void {
        this->ilotCacheLock.reset();
    };

}