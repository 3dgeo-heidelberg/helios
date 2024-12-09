#include <PySceneWrapper.h>
#include <PyHeliosException.h>

using pyhelios::PySceneWrapper;

// ***   METHODS   *** //
// ******************* //
void PySceneWrapper::translate(double const x, double const y, double const z){
    glm::dvec3 const v(x, y, z);
    for(Primitive *p : scene.primitives) p->translate(v);
}

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
