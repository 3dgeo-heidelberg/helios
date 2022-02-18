#include <GroveKDTreeRaycaster.h>
#include <Primitive.h>

#include <vector>

using std::vector;

// ***  GROVE DYNAMIC TREE METHODS  *** //
// ************************************ //
void GroveKDTreeRaycaster::update(DynObject &dynObj){
    // Copy primitives
    vector<Primitive *> prims;
    for(Primitive * prim : dynObj.mPrimitives){
        Primitive * clone = prim->clone();
        clone->part = prim->part;
        prims.push_back(clone);
    }

    // Make new tree with its independent set of primitives
    // To prevent they from being updated by other threads when raycasting
    root = shared_ptr<LightKDTreeNode>(
        kdtf->makeFromPrimitives(prims, true, false)
    );
}

std::shared_ptr<GroveKDTreeRaycaster> GroveKDTreeRaycaster::makeTemporalClone()
const{
    return std::make_shared<GroveKDTreeRaycaster>(root);
}
