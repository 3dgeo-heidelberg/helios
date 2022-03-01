#include <scene/dynamic/DynSequentiableMovingObject.h>

#include <vector>

using std::vector;

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
bool DynSequentiableMovingObject::doSimStep(){
    // Fill motion queues from sequencer
    fillMotionQueues();

    // Return DynMovingObject output
    return DynMovingObject::doSimStep();
}

void DynSequentiableMovingObject::fillMotionQueues(){
    // Fill motion queues from sequencer
    if(dmSequencer.hasNextStep()){ // If there is a next sequence
        // Iterate over its dynamic motions
        vector<shared_ptr<DynMotion>> sequence = dmSequencer.nextStep();
        for(shared_ptr<DynMotion> &dm : sequence){
            pushPositionMotion(dm); // Push dynamic motion to position queue
            if(dm->checkModifiesNormal()){
                // If dynamic motion modifies normal, push to normal queue too
                // Considering only the fixed transformation with zero shift
                pushNormalMotion(dm->makeNormalCounterpartPtr());
            }
        }
    }
}
