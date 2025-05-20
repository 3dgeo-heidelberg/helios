#ifndef _SERIAL_ADT_UTILS
#define _SERIAL_ADT_UTILS

#include <AxisSAHKDTreeGeometricStrategy.h>
#include <FastSAHKDTreeGeometricStrategy.h>
#include <MultiThreadKDTreeFactory.h>
#include <SAHKDTreeGeometricStrategy.h>
#include <SimpleKDTreeGeometricStrategy.h>

#include <memory>

// ***  UTIL FUNCTIONS  *** //
// ************************ //
char
KDTREE_FACTORY_EXTRACT_GSTYPE(MultiThreadKDTreeFactory const* mtkdtf);

template<typename T>
void
KDTREE_FACTORY_INPLACE_CONSTRUCT(T* mtkdtf,
                                 std::shared_ptr<SimpleKDTreeFactory> kdtf,
                                 size_t const numJobs,
                                 size_t const geomJobs,
                                 char const gsType)
{
  if (gsType == 1) {
    std::shared_ptr<SimpleKDTreeGeometricStrategy> gs =
      std::make_shared<SimpleKDTreeGeometricStrategy>(*kdtf);
    ::new (mtkdtf) T(kdtf, gs, numJobs, geomJobs);
  } else if (gsType == 2) {
    std::shared_ptr<SAHKDTreeGeometricStrategy> gs =
      std::make_shared<SAHKDTreeGeometricStrategy>(
        *std::dynamic_pointer_cast<SAHKDTreeFactory>(kdtf));
    ::new (mtkdtf) T(kdtf, gs, numJobs, geomJobs);
  } else if (gsType == 3) {
    std::shared_ptr<AxisSAHKDTreeGeometricStrategy> gs =
      std::make_shared<AxisSAHKDTreeGeometricStrategy>(
        *std::dynamic_pointer_cast<AxisSAHKDTreeFactory>(kdtf));
    ::new (mtkdtf) T(kdtf, gs, numJobs, geomJobs);
  } else if (gsType == 4) {
    std::shared_ptr<FastSAHKDTreeGeometricStrategy> gs =
      std::make_shared<FastSAHKDTreeGeometricStrategy>(
        *std::dynamic_pointer_cast<FastSAHKDTreeFactory>(kdtf));
    ::new (mtkdtf) T(kdtf, gs, numJobs, geomJobs);
  } else {
    ::new (mtkdtf) T(kdtf, nullptr, numJobs, geomJobs);
  }
}

#endif
