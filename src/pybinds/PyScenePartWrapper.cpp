#ifdef PYTHON_BINDING

#include <PyScenePartWrapper.h>
#include <PyHeliosException.h>
#include <PyPrimitiveWrapper.h>
#include <Primitive.h>

using pyhelios::PyScenePartWrapper;
using pyhelios::PyPrimitiveWrapper;

// ***   UTIL METHODS   *** //
// ************************ //
void PyScenePartWrapper::translate(double const x, double const y, double const z){
    glm::dvec3 const v(x, y, z);
    for(Primitive *p : sp.mPrimitives) p->translate(v);
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
PyPrimitiveWrapper * PyScenePartWrapper::getPrimitive(size_t const index){
    return new PyPrimitiveWrapper(sp.mPrimitives[index]);
}

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

#endif