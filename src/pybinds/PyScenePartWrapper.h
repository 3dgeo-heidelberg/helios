#pragma once

#include <string>
#include <PythonDVec3.h>
#include <ScenePart.h>
#include <DynMovingObject.h>

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
    bool isDynamicMovingObject()
    {return sp.getType() == ScenePart::ObjectType::DYN_MOVING_OBJECT;}
    size_t getDynObjectStep() {return _asDynObject().getStepInterval();}
    void setDynObjectStep(size_t const stepInterval)
    {return _asDynObject().setStepInterval(stepInterval);}
    size_t getObserverStep()
    {return _asDynMovingObject().getObserverStepInterval();}
    void setObserverStep(size_t const stepInterval)
    {_asDynMovingObject().setObserverStepInterval(stepInterval);}


    // ***  INTERNAL USE  *** //
    // ********************** //
    /**
     * @brief Obtain the scene part as a dynamic object. Use with caution as
     *  it might throw an exception
     */
    DynObject & _asDynObject();
    /**
     * @brief Obtain the scene part as a dynamic moving object. Use with
     *  caution as it might throw an exception
     */
    DynMovingObject & _asDynMovingObject();
};

}
