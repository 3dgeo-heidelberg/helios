#ifdef PCL_BINDING

#include <VHDynObjectAdapter.h>

using visualhelios::VHDynObjectAdapter;

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
bool VHDynObjectAdapter::doStep(bool const forceStep, bool const forceRebuild){
    bool updated = false;
    if(forceStep) updated = getDynObj().doStep();
    updated |= forceRebuild;
    if(updated) buildPolymesh();
    return updated;
}

#endif