#include <GroveKDTreeRaycaster.h>

// ***  GROVE DYNAMIC TREE METHODS  *** //
// ************************************ //
void GroveKDTreeRaycaster::update(DynObject &dynObj){
    // TODO Rethink : Implement
    root = shared_ptr<LightKDTreeNode>(
        kdtf->makeFromPrimitives(dynObj.mPrimitives, true, false)
    );
    std::cout << "DynObject was updated!" << std::endl; // TODO Remove
}

std::shared_ptr<GroveKDTreeRaycaster> GroveKDTreeRaycaster::makeTemporalClone()
const{
    return std::make_shared<GroveKDTreeRaycaster>(root);
}
