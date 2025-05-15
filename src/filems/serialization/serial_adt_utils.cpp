#include <serial_adt_utils.h>

// ***  UTIL FUNCTIONS  *** //
// ************************ //
char
KDTREE_FACTORY_EXTRACT_GSTYPE(MultiThreadKDTreeFactory const* mtkdtf)
{
  char gsType = 1; // By default : SimpleKDTreeGeometricStratey (1)
  if (mtkdtf->getGS() == nullptr) { // No geometric strategy : 0
    gsType = 0;
  } else if (dynamic_pointer_cast<FastSAHKDTreeGeometricStrategy>(
               mtkdtf->getGS()) !=
             nullptr) { // Fast SAH KDTree geometric strategy : 4
    gsType = 4;
  } else if (dynamic_pointer_cast<AxisSAHKDTreeGeometricStrategy>(
               mtkdtf->getGS()) !=
             nullptr) { // Axis SAH KDTree geometric strategy : 3
    gsType = 3;
  } else if (dynamic_pointer_cast<SAHKDTreeGeometricStrategy>(
               mtkdtf->getGS()) !=
             nullptr) { // SAH KDTree geometric strategy : 2
    gsType = 2;
  }
  return gsType;
}
