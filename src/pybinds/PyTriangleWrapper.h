#pragma once

#ifdef PYTHON_BINDING

#include <PyPrimitiveWrapper.h>
#include <Triangle.h>

namespace pyhelios{

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
    inline PythonDVec3 * getFaceNormal()
        {return new PythonDVec3( ((Triangle *) prim)->getFaceNormal() ); }
    // ***  TO STRING  *** //
    // ******************* //
    inline std::string toString(){return ((Triangle *) prim)->toString();}
};

}

#endif