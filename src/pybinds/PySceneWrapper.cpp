#ifdef PYTHON_BINDING
#include <PySceneWrapper.h>
#include <PyHeliosException.h>

using pyhelios::PySceneWrapper;

// ***  INTERNAL USE  *** //
// ********************** //
DynScene & PySceneWrapper::_asDynScene(){
    try{
        return dynamic_cast<DynScene &>(scene);
    }
    catch(std::exception &ex){
        throw PyHeliosException(
            "Failed to obtain scene as dynamic scene"
        );
    }
}

#endif