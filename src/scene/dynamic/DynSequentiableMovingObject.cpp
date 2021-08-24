#include <scene/dynamic/DynSequentiableMovingObject.h>

#include <vector>

using std::vector;

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
bool DynSequentiableMovingObject::doStep(){
    // Fill motion queues from sequencer
    fillMotionQueues();

    // Return DynMovingObject output
    return DynMovingObject::doStep();
}

void DynSequentiableMovingObject::fillMotionQueues(){
    // Fill motion queues from sequencer
    if(rmSequencer.hasNextStep()){ // If there is a next sequence
        // Iterative over its rigid motions
        vector<shared_ptr<RigidMotion>> sequence = rmSequencer.nextStep();
        for(shared_ptr<RigidMotion> rm : sequence){
            pushPositionMotion(rm); // Push rigid motion to position queue
            RigidMotion::SuperType stype = rm->findSuperType();
            if( stype == RigidMotion::SuperType::R2_REFLECTION ||
                stype == RigidMotion::SuperType::R2_ROTATION ||
                stype == RigidMotion::SuperType::R3_REFLECTION ||
                stype == RigidMotion::SuperType::R3_ROTATION ||
                stype == RigidMotion::SuperType::R3_ROTATIONAL_SYMMETRY
            ){ // If rigid motion modifies normal, push to normal queue too
                // Considering only the fixed transformation with zero shift
                pushNormalMotion(make_shared<RigidMotion>(
                    arma::zeros(rm->getDimensionality()),
                    rm->getA()
                ));
            }
        }
    }
}
