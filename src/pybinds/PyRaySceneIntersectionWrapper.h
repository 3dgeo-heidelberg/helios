#pragma once

#ifdef PYTHON_BINDING

#include <PythonDVec3.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for RaySceneIntersection
 *
 * @see RaySceneIntersection
 */
class PyRaySceneIntersectionWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    RaySceneIntersection * rsi;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyRaySceneIntersectionWrapper(RaySceneIntersection const rsi) :
        rsi(new RaySceneIntersection(rsi)) {}
    virtual ~PyRaySceneIntersectionWrapper() {delete rsi;}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    PyPrimitiveWrapper * getPrimitive()
        {return new PyPrimitiveWrapper(rsi->prim);}
    PythonDVec3 * getPoint()
        {return new PythonDVec3(rsi->point);}
    double getIncidenceAngle() {return rsi->incidenceAngle;}
    void setIncidenceAngle(double incidenceAngle)
        {rsi->incidenceAngle = incidenceAngle;}

};

#endif