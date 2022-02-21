#pragma once

#ifdef PYTHON_BINDING

#include <PyVertexWrapper.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for AABB class
 *
 * @see AABB
 */
class PyAABBWrapper{
public:
    // ***  ATTRIBUTE  *** //
    // ******************* //
    AABB *aabb;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyAABBWrapper(AABB *aabb) : aabb(aabb) {}
    virtual ~PyAABBWrapper() = default;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    inline PyVertexWrapper * getMinVertex()
        {return new PyVertexWrapper(aabb->vertices);}
    inline PyVertexWrapper * getMaxVertex()
        {return new PyVertexWrapper(aabb->vertices + 1);}

    // ***  TO STRING  *** //
    // ******************* //
    inline std::string toString(){return aabb->toString();}
};

}

#endif