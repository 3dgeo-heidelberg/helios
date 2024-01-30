#pragma once

#include <string>
#include <Triangle.h>
#include <DetailedVoxel.h>
#include <AABB.h>
#include <Scene.h>
#include <DynScene.h>
#include <PyPrimitiveWrapper.h>
#include <PyTriangleWrapper.h>
#include <PyDetailedVoxelWrapper.h>
#include <PyAABBWrapper.h>
#include <PyRaySceneIntersectionWrapper.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Scene
 *
 * @see Scene
 */
class PySceneWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    Scene &scene;

    // ***  CONSTRUCTOR / DESTRUCTOR  *** //
    // ********************************** //
    PySceneWrapper(Scene &scene) : scene(scene) {}
    virtual ~PySceneWrapper() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    PyTriangleWrapper * newTriangle(){
        Vertex v;
        v.pos[0] = 0.0;     v.pos[1] = 0.0;     v.pos[2] = 0.0;
        Triangle * tri = new Triangle(v, v, v);
        scene.primitives.push_back(tri);
        return new PyTriangleWrapper(tri);
    }
    PyDetailedVoxelWrapper * newDetailedVoxel(){
        std::vector<int> vi({0, 0});
        std::vector<double> vd({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
        DetailedVoxel *dv = new DetailedVoxel(0.0, 0.0, 0.0, 0.5, vi, vd);
        scene.primitives.push_back(dv);
        return new PyDetailedVoxelWrapper(dv);
    }
    PyPrimitiveWrapper * getPrimitive(size_t index)
        {return new PyPrimitiveWrapper(scene.primitives[index]);}
    PyAABBWrapper * getAABB()
        {return new PyAABBWrapper(scene.getAABB().get());}
    PythonDVec3 * getGroundPointAt(double x, double y, double z){
        glm::dvec3 gp = glm::dvec3(x, y, z);
        return new PythonDVec3(gp);
    }
    PyRaySceneIntersectionWrapper * getIntersection(
        double ox, double oy, double oz, // Origin
        double dx, double dy, double dz, // Direction
        bool groundOnly
    ){
        glm::dvec3 origin(ox, oy, oz);
        glm::dvec3 direction(dx, dy, dz);
        return new PyRaySceneIntersectionWrapper(
            *scene.getIntersection(origin, direction, groundOnly)
        );
    }
    PythonDVec3 * getShift(){return new PythonDVec3(scene.getShift());}
    size_t getNumSceneParts(){return scene.parts.size();}
    PyScenePartWrapper * getScenePart(size_t const i)
    {return new PyScenePartWrapper(*scene.parts[i]);}
    size_t getDynSceneStep(){return _asDynScene().getStepInterval();}
    void setDynSceneStep(size_t const stepInterval)
    {_asDynScene().setStepInterval(stepInterval);}


    // ***  M E T H O D S  *** //
    // *********************** //
    bool finalizeLoading() {return scene.finalizeLoading();}
    void writeObject(std::string path) {scene.writeObject(path);}

    // ***  INTERNAL USE  *** //
    // ********************** //
    /**
     * @brief Obtain the scene as a dynamic scene if possible. Use with caution
     *  because an exception can be thrown if the scene is not dynamic.
     */
    DynScene & _asDynScene();
};

}
