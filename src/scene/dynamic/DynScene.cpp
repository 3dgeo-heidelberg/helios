#include <scene/dynamic/DynScene.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
DynScene::DynScene(DynScene &ds) : DynScene(static_cast<Scene&>(ds)){
    for(shared_ptr<DynObject> dynObj : ds.dynObjs){
        dynObjs.push_back(dynObj);
    }
}

// ***  SIMULATION STEP  *** //
// ************************* //
void DynScene::doSimStep(){
    currentStep = (currentStep + 1) % dynamicSpaceInterval;
    if(currentStep == (dynamicSpaceInterval-1)) doStep();
}

void DynScene::doStep(){
    for(shared_ptr<DynObject> & dynObj : dynObjs) dynObj->doStep();
}
