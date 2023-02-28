#ifdef PCL_BINDING
#ifndef _VHDYNSCENEADAPTER_H_

using std::make_shared;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
template <typename ST, typename DT>
VHDynSceneAdapter<ST, DT>::VHDynSceneAdapter(DynScene &dynScene) :
    dynScene(dynScene)
{
    // Add static objects
    size_t const m = dynScene.numStaticObjects();
    for(size_t i = 0 ; i < m ; ++i){
        staticObjs.push_back(make_shared<ST>(*dynScene.getStaticObject(i)));
    }

    // Add dynamic objects
    size_t const n = dynScene.numDynObjects();
    for(size_t i = 0 ; i < n ; ++i){
        dynObjs.push_back(make_shared<DT>(*dynScene.getDynObject(i)));
    }
}

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
template <typename ST, typename DT>
bool VHDynSceneAdapter<ST, DT>::doStep(){
    // Compute steps
    bool updated = dynScene.doSimStep();

    // Rebuild polygon mesh for updated objects
    size_t const m = numDynObjects();
    for(size_t i = 0 ; i < m ; ++i){
        if(isDynObjectUpdated(i)) getAdaptedDynObj(i)->doStep(false, true);
    }

    // Return update status
    return updated;
}

#endif
#endif