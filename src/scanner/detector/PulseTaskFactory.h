#pragma once

#include <PulseTask.h>
#include <Scene.h>
#include <SimulatedPulse.h>
class ScanningPulseProcess;

#include <functional>
#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Factory to make the adequate pulse task depending on the simulation
 * @see PulseTask
 * @see FullWaveformPulseRunnable
 * @see DynFullWaveformPulseRunnable
 */
class PulseTaskFactory
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Reference to the scene to be scanned
   */
  Scene& scene;
  /**
   * @brief The build function to be used.
   *
   * It depends on the simulation. For instance, if there are dynamic
   *  moving objects, it will build a DynFullWaveformPulseRunnable. But
   *  if there is no dynamic object at all, then a FullWaveformPulseRunnable
   *  will be built.
   */
  std::function<std::shared_ptr<PulseTask>(ScanningPulseProcess const&,
                                           SimulatedPulse const&)>
    _build;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for the PulseTaskFactory
   * @see PulseTaskFactory::scene
   */
  PulseTaskFactory(Scene& scene);
  virtual ~PulseTaskFactory() {}

  // ***  FACTORY METHODS  *** //
  // ************************* //
  /**
   * @brief The main build method of the factory. It is the method that must
   *  be called to automatically build the adequate type of PulseTask
   * @return Built pulse task
   * @see PulseTask
   */
  inline std::shared_ptr<PulseTask> build(ScanningPulseProcess const& spp,
                                          SimulatedPulse const& sp) const
  {
    return _build(spp, sp);
  }

protected:
  /**
   * @brief Build a FullWaveformPulseRunnable from given arguments
   * @see PulseTaskFactory::build
   */
  std::shared_ptr<PulseTask> buildFullWaveformPulseRunnable(
    ScanningPulseProcess const& spp,
    SimulatedPulse const& sp) const;
  /**
   * @brief Build a DynFullWaveformPulseRunnable from given arguments
   * @see PulseTaskFactory::build
   */
  std::shared_ptr<PulseTask> buildDynFullWaveformPulseRunnable(
    ScanningPulseProcess const& spp,
    SimulatedPulse const& sp) const;

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
  inline Scene& getScene() const { return scene; }
  /**
   * @brief Set the scene associated to the pulse task factory
   * @param scene Scene associated to the pulse task factory
   * @see PulseTaskFactory::scene
   */
  inline void setScene(Scene& scene)
  {
    this->scene = scene;
    configureBuildMethod();
  }
};
