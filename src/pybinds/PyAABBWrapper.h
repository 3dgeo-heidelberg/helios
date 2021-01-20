#pragma once

#ifdef PYTHON_BINDING

#include <PyVertexWrapper.h>

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
    PyVertexWrapper * getMinVertex()
        {return new PyVertexWrapper(aabb->vertices);}
    PyVertexWrapper * getMaxVertex()
        {return new PyVertexWrapper(aabb->vertices + 1);}
};

#endif