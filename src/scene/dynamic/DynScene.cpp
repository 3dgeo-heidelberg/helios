#include <scene/dynamic/DynScene.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
DynScene::DynScene(DynScene &ds) :
    DynScene(static_cast<StaticScene&>(ds))
{
    for(shared_ptr<DynObject> dynObj : ds.dynObjs){
        dynObjs.push_back(dynObj);
        updated.push_back(true);
    }
    dynamicSpaceInterval = ds.dynamicSpaceInterval;
    currentStep = ds.currentStep;
}

// ***  SIMULATION STEP  *** //
// ************************* //
bool DynScene::doSimStep(){
    currentStep = (currentStep + 1) % dynamicSpaceInterval;
    if(currentStep == (dynamicSpaceInterval-1)) return doStep();
    return false;
}

bool DynScene::doStep(){
    bool updateFlag = false;
    size_t const n = numDynObjects();
    for(size_t i = 0 ; i < n ; ++i){
        updated[i] = dynObjs[i]->doStep();
        updateFlag |= updated[i];
    }
    return updateFlag;
}
