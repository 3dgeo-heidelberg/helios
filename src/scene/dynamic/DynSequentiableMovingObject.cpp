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


// ***   U T I L S   *** //
// ********************* //
void DynSequentiableMovingObject::applyAutoCRS(
    double const x, double const y, double const z
){
    unordered_map<string, shared_ptr<DynSequence<DynMotion>>> const &dynseqs =
        dmSequencer.getAllSequencesByRef();
    unordered_map<string, shared_ptr<DynSequence<DynMotion>>>::const_iterator
        it;
    arma::colvec u({x, y, z});
    for(it = dynseqs.begin() ; it != dynseqs.end() ; ++it){
        shared_ptr<DynSequence<DynMotion>> ds = it->second;
        size_t const numMotions = ds->size();
        for(size_t i = 0 ; i < numMotions ; ++i){
            shared_ptr<DynMotion> dm = ds->get(i);
            if(
                dm->findType() == DynMotion::Type::TRANSLATION_R3 &&
                dm->isAutoCRS()
            ) {
                dm->setC(dm->getC()+dm->getAutoCRS()*u);
            }
        }
    }
}
