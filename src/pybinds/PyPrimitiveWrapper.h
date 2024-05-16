#pragma once

#ifdef PYTHON_BINDING

#include <PyScenePartWrapper.h>
#include <PyAABBWrapper.h>
#include <PythonDVec3.h>
#include <PyDoubleVector.h>
#include <PyVertexWrapper.h>
#include <Triangle.h>
#include <AABB.h>
#include <Voxel.h>
#include <DetailedVoxel.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Primitive class
 *
 * @see Primitive
 */
class PyPrimitiveWrapper{
public:
    // ***  ATTRIBUTE  *** //
    // ******************* //
    Primitive *prim = nullptr;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyPrimitiveWrapper(Primitive *prim) : prim(prim) {}
    virtual ~PyPrimitiveWrapper() = default;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    PyScenePartWrapper * getScenePart()
        {return new PyScenePartWrapper(*prim->part);}
    Material & getMaterial()
        {return *prim->material;}
    PyAABBWrapper * getAABB() {return new PyAABBWrapper(prim->getAABB());}
    PythonDVec3 * getCentroid() {return new PythonDVec3(prim->getCentroid());}
    double getIncidenceAngle(
        double ox, double oy, double oz,
        double dx, double dy, double dz,
        double px, double py, double pz
    ){
        glm::dvec3 origin(ox, oy, oz);
        glm::dvec3 direction(dx, dy, dz);
        glm::dvec3 intersectionPoint(px, py, pz);
        return prim->getIncidenceAngle_rad(
            origin,
            direction,
            intersectionPoint
        );
    }
    PyDoubleVector * getRayIntersection(
        double ox, double oy, double oz,
        double dx, double dy, double dz
    ){
        glm::dvec3 origin(ox, oy, oz);
        glm::dvec3 direction(dx, dy, dz);
        return new PyDoubleVector(prim->getRayIntersection(origin, direction));
    }
    double getRayIntersectionDistance(
        double ox, double oy, double oz,
        double dx, double dy, double dz
    ){
        glm::dvec3 origin(ox, oy, oz);
        glm::dvec3 direction(dx, dy, dz);
        return prim->getRayIntersectionDistance(origin, direction);
    }
    size_t getNumVertices(){return prim->getNumVertices();}
    PyVertexWrapper * getVertex(size_t index)
        {return new PyVertexWrapper(prim->getVertices()+index);}
    bool isTriangle () const
        {return dynamic_cast<Triangle *>(prim) != nullptr;}
    bool isAABB () const
        {return dynamic_cast<AABB *>(prim) != nullptr;}
    bool isVoxel () const
        {return dynamic_cast<Voxel *>(prim) != nullptr;}
    bool isDetailedVoxel () const
        {return dynamic_cast<DetailedVoxel *>(prim) != nullptr;}
    void update(){prim->update();}


};

}

#endif