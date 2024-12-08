#pragma once

#include <Vertex.h>
#include <PythonDVec3.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Vertex class
 *
 * @see Vertex
 */
class PyVertexWrapper{
public:
    // ***  ATTRIBUTE  *** //
    // ******************* //
    Vertex *v;
    bool release = true;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyVertexWrapper(Vertex *v){
        this->v = v;
        release = false;
    }
    PyVertexWrapper(Vertex const v){
        this->v = new Vertex(v);
        release = true;
    }
    virtual ~PyVertexWrapper(){}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    PythonDVec3 * getPosition() {return new PythonDVec3(&v->pos);}
    PythonDVec3 * getNormal() {return new PythonDVec3(&v->normal);}
};

}
