#include <MultiThreadSAHKDTreeFactory.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
MultiThreadSAHKDTreeFactory::MultiThreadSAHKDTreeFactory(
  shared_ptr<SimpleKDTreeFactory> const kdtf,
  shared_ptr<SimpleKDTreeGeometricStrategy> const gs,
  size_t const numJobs,
  size_t const geomJobs)
  : MultiThreadKDTreeFactory(kdtf, gs, numJobs, geomJobs)
{
  // sahkdtf by casting kdtf
  sahkdtf = std::static_pointer_cast<SAHKDTreeFactory>(kdtf);

  // Replace sahkdtf ILOT lock/unlock functions
  sahkdtf->_lockILOT = [&]() -> void {
    this->ilotCacheLock.reset(
      new boost::unique_lock<boost::mutex>(this->ilotCacheMutex));
  };
  sahkdtf->_unlockILOT = [&]() -> void { this->ilotCacheLock.reset(); };
}

// ***  CLONE  *** //
// *************** //
KDTreeFactory*
MultiThreadSAHKDTreeFactory::clone() const
{
  shared_ptr<SimpleKDTreeFactory> skdtf((SimpleKDTreeFactory*)kdtf->clone());
  MultiThreadSAHKDTreeFactory* mtkdtf = new MultiThreadSAHKDTreeFactory(
    skdtf,
    shared_ptr<SimpleKDTreeGeometricStrategy>(gs->clone(skdtf.get())),
    numJobs,
    geomJobs);
  _clone(mtkdtf);
  return mtkdtf;
}

void
MultiThreadSAHKDTreeFactory::_clone(KDTreeFactory* kdtf) const
{
  MultiThreadKDTreeFactory::_clone(kdtf);
}
