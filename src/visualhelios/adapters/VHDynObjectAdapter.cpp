#ifdef PCL_BINDING

#include <VHDynObjectAdapter.h>

using visualhelios::VHDynObjectAdapter;

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
bool VHDynObjectAdapter::doStep(){
    bool updated = getDynObj().doStep();
    if(updated) buildPolymesh();
    return updated;
}

#endif