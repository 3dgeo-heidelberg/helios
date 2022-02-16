#pragma once

#include <PulseTask.h>
#include <Scene.h>
class ScanningPulseProcess;

#include <functional>
#include <memory>

using std::shared_ptr;
using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Factory to make the adequate pulse task depending on the simulation
 * @see PulseTask
 * @see FullWaveformPulseRunnable
 * @see DynFullWaveformPulseRunnable
 */
class PulseTaskFactory{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Reference to the scene to be scanned
     */
    Scene &scene;
    /**
     * @brief The build function to be used.
     *
     * It depends on the simulation. For instance, if there are dynamic
     *  moving objects, it will build a DynFullWaveformPulseRunnable. But
     *  if there is no dynamic object at all, then a FullWaveformPulseRunnable
     *  will be built.
     */
    std::function<shared_ptr<PulseTask>(
        ScanningPulseProcess const &,
        unsigned int const,
        glm::dvec3 &,
        Rotation &,
        double const
    )> _build;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for the PulseTaskFactory
     * @see PulseTaskFactory::scene
     */
    PulseTaskFactory(Scene &scene);
    virtual ~PulseTaskFactory() {}

    // ***  FACTORY METHODS  *** //
    // ************************* //
    /**
     * @brief The main build method of the factory. It is the method that must
     *  be called to automatically build the adequate type of PulseTask
     * @return Built pulse task
     * @see PulseTask
     */
    inline shared_ptr<PulseTask> build(
        ScanningPulseProcess const &spp,
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    ) const {return _build(
        spp, legIndex, absoluteBeamOrigin, absoluteBeamAttitude, currentGpsTime
    );}

protected:
    /**
     * @brief Build a FullWaveformPulseRunnable from given arguments
     * @see PulseTaskFactory::build
     */
    shared_ptr<PulseTask> buildFullWaveformPulseRunnable(
        ScanningPulseProcess const &spp,
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    ) const;
    /**
     * @brief Build a DynFullWaveformPulseRunnable from given arguments
     * @see PulseTaskFactory::build
     */
    shared_ptr<PulseTask> buildDynFullWaveformPulseRunnable(
        ScanningPulseProcess const &spp,
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    ) const;

    /**
     * @brief Automatically set the _build function depending on current
     *  factory state
     * @see PulseTaskFactory::_build
     */
    virtual void configureBuildMethod();

public:
    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the scene associated to the pulse task factory
     * @return Scene associated to the pulse task factory
     * @see PulseTaskFactory::scene
     */
    inline Scene & getScene() const {return scene;}
    /**
     * @brief Set the scene associated to the pulse task factory
     * @param scene Scene associated to the pulse task factory
     * @see PulseTaskFactory::scene
     */
    inline void setScene(Scene &scene)
    {this->scene = scene; configureBuildMethod();}

};