#pragma once

#ifdef PYTHON_BINDING

#include <string>
#include <ScenePart.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for ScenePart class
 *
 * @see ScenePart
 */
class PyScenePartWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    ScenePart &sp;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyScenePartWrapper(ScenePart &sp) : sp(sp) {}
    virtual ~PyScenePartWrapper() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    std::string getId() {return sp.mId;};
    void setId(std::string id) {sp.mId = id;}
    PythonDVec3 * getOrigin() {return new PythonDVec3(sp.mOrigin);}
    void setOrigin(double x, double y, double z)
        {sp.mOrigin = glm::dvec3(x, y, z);}
    Rotation & getRotation() {return sp.mRotation;}
    void setRotation(double x, double y, double z, double angle)
        {sp.mRotation = Rotation(glm::dvec3(x, y, z), angle);}
    double getScale() {return sp.mScale;}
    void setScale(double scale) {sp.mScale = scale;}
};

}

#endif