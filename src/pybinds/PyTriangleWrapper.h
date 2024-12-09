#pragma once

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
    ~PyTriangleWrapper() override = default;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    inline PythonDVec3 * getFaceNormal()
        {return new PythonDVec3( ((Triangle *) prim)->getFaceNormal() ); }
    // ***  TO STRING  *** //
    // ******************* //
    inline std::string toString(){return ((Triangle *) prim)->toString();}
};

}
