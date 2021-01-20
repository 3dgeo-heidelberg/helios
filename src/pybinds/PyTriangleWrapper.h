#pragma once

#ifdef PYTHON_BINDING

#include <PyPrimitiveWrapper.h>
#include <Triangle.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Triangle class
 *
 * @see Triangle
 */
class PyTriangleWrapper : public PyPrimitiveWrapper {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyTriangleWrapper(Triangle * tri) : PyPrimitiveWrapper(tri) {}
    virtual ~PyTriangleWrapper() = default;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    PythonDVec3 * getFaceNormal()
        {return new PythonDVec3( ((Triangle *) prim)->getFaceNormal() ); }
};

#endif