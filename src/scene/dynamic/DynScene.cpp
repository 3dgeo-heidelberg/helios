#include <scene/dynamic/DynScene.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
DynScene::DynScene(DynScene &ds) : Scene(ds){
    for(shared_ptr<DynObject> dynObj : ds.dynObjs){
        dynObjs.push_back(dynObj);
    }
}

// ***  SIMULATION STEP  *** //
// ************************* //
void DynScene::doSimStep(){
    currentStep = (currentStep + 1) % dynamicSpaceFrequency;
    if(currentStep == (dynamicSpaceFrequency-1)) doStep();
}

void DynScene::doStep(){
    for(shared_ptr<DynObject> & dynObj : dynObjs) dynObj->doStep();
}
