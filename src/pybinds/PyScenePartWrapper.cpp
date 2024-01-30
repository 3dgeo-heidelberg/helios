#include <PyScenePartWrapper.h>
#include <PyHeliosException.h>

using pyhelios::PyScenePartWrapper;

// ***  INTERNAL USE  *** //
// ********************** //
DynObject & PyScenePartWrapper::_asDynObject(){
    try{
        return dynamic_cast<DynObject&>(sp);
    }
    catch(std::exception &ex){
        throw PyHeliosException(
            "Failed to retrieve scene part as dynamic object"
        );
    }
}
DynMovingObject & PyScenePartWrapper::_asDynMovingObject(){
    try{
        return dynamic_cast<DynMovingObject&>(sp);
    }
    catch(std::exception &ex){
        throw PyHeliosException(
            "Failed to retrieve scene part as dynamic moving object"
        );
    }
}
