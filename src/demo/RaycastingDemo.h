#ifdef PCL_BINDING
#pragma once

#include <demo/DynamicSceneDemo.h>
#include <Platform.h>

// TODO Rethink : Implement this demo
namespace HeliosDemos{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Raycasting demo
 *
 * This demo extends the DynamicSceneDemo to also render the rays defining the
 *  scanning process simulation
 *
 * @see HeliosDemos::DynamicSceneDemo
 */
class RaycastingDemo : public DynamicSceneDemo{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The scanner emitting the rays
     */
    shared_ptr<Scanner> scanner = nullptr;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Ray casting demo constructor
     * @see HeliosDemos::DynamicSceneDemo::DynamicSceneDemo
     */
    RaycastingDemo(string const surveyPath, string const assetsPath) :
        DynamicSceneDemo("Raycasting demo", surveyPath, assetsPath)
    {}
    virtual ~DynamicSceneDemo() = default;
};

}

#endif