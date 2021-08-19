#ifdef PCL_BINDING
#ifndef _VHDYNSCENEADAPTER_H_

using std::make_shared;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
template <typename T>
VHDynSceneAdapter<T>::VHDynSceneAdapter(DynScene &dynScene):dynScene(dynScene){
    size_t n = dynScene.numDynObjects();
    for(size_t i = 0 ; i < n ; ++i){
        dynObjs.push_back(make_shared<T>(*dynScene.getDynObject(i)));
    }
}

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
template <typename T>
bool VHDynSceneAdapter<T>::doStep(){
    bool updated = false;
    for(shared_ptr<T> dynObj : dynObjs){
        updated |= static_pointer_cast<VHDynObjectAdapter>(dynObj)->doStep();
    }
    return updated;
}

#endif
#endif