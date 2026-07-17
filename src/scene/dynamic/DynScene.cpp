#include <logging.hpp>
#include <scene/dynamic/DynScene.h>
#include <scene/primitives/AABB.h>

using std::stringstream;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
DynScene::DynScene(DynScene& ds)
  : DynScene(static_cast<StaticScene&>(ds))
{
  for (std::shared_ptr<DynObject> dynObj : ds.dynObjs) {
    dynObjs.push_back(dynObj);
    updated.push_back(true);
  }
  makeStepLoop(ds.stepLoop.getStepInterval());
  stepLoop.setCurrentStep(ds.stepLoop.getCurrentStep());
  dynTimeStep = ds.dynTimeStep;
}

// ***   M E T H O D S   *** //
// ************************* //
void
DynScene::prepareSimulation(int const simFrequency_hz)
{
  if (!simulationPrepared) {
    hasUnresettableSwapOnRepeat = !getSwapOnRepeatObjects().empty();
  } else if (hasUnresettableSwapOnRepeat) {
    throw HeliosException(
      "Repeated playback of a dynamic scene containing swap-on-repeat "
      "handlers is not supported because swap filter recipes are consumed "
      "during the first playback; reload the scene from XML.");
  }

  resetSimulationState();
  simulationPrepared = true;

  // Prepare variables
  double const simFreq_hz = (double)simFrequency_hz;
  // Configure the dynamic scene step interval from time
  // (overrides the previous discrete step configuration)
  if (!std::isnan(dynTimeStep)) {
    if (dynTimeStep > 1.0) {
      std::stringstream ss;
      ss << "DynScene::prepareSimulation detected the scene's dynamic "
            "time step ("
         << dynTimeStep
         << ") is greater than one. "
            "This is not supported. "
            "Any dynamic time step must be inside (0, 1].";
      logging::WARN(ss.str());
    }
    setStepInterval((int)(simFreq_hz * dynTimeStep));
    // Configure each dynamic object step interval from time
    for (std::shared_ptr<DynObject>& dynObj : dynObjs) {
      // Configure the dynamic step interval for each part from time
      if (std::isnan(dynObj->getDynTimeStep())) {
        dynObj->setDynTimeStep(dynTimeStep); // Part from scene
      }
      double const partDt = dynObj->getDynTimeStep();
      if (partDt > 1.0) {
        std::stringstream ss;
        ss << "DynScene::prepareSimulation detected a dynamic object "
              "with dynamic time step ("
           << partDt
           << ") greater than "
              "one. This is not supported. "
              "Any dynamic time step must be inside (0, 1].";
        logging::WARN(ss.str());
      }
      dynObj->setStepInterval((int)(partDt / dynTimeStep));
      // Configure the dynamic step interval for each observer from time
      std::shared_ptr<DynMovingObject> obs =
        std::dynamic_pointer_cast<DynMovingObject>(dynObj);
      if (std::isnan(obs->getObserverDynTimeStep())) {
        obs->setObserverDynTimeStep(partDt);
      }
      double const kdtDt = obs->getObserverDynTimeStep();
      if (kdtDt > 1.0) {
        std::stringstream ss;
        ss << "DynScene::prepareSimulation detected a DynKDT "
              "with dynamic time step ("
           << kdtDt
           << ") greater than "
              "one. This is not supported. "
              "Any dynamic time step must be inside (0, 1].";
        logging::WARN(ss.str());
      }
      obs->setObserverStepInterval((int)(kdtDt / partDt));
    }
  }
}

void
DynScene::resetSimulationState()
{
  stepLoop.setCurrentStep(0);
  std::fill(updated.begin(), updated.end(), true);

  glm::dvec3 const sceneShift = getShift();
  for (std::shared_ptr<DynObject>& dynObj : dynObjs)
    dynObj->resetSimulationState(sceneShift);

  // Restore the scene-level bounds without inspecting or copying geometry.
  bbox = std::make_shared<AABB>(bbox_crs->getMin() - sceneShift,
                                bbox_crs->getMax() - sceneShift);

  // Observer intervals can intentionally leave a stale dynamic tree at the
  // end of a run. Bypass those intervals so the first ray of the next run
  // always sees restored geometry.
  for (std::shared_ptr<DynObject>& dynObj : dynObjs) {
    std::shared_ptr<DynMovingObject> moving =
      std::dynamic_pointer_cast<DynMovingObject>(dynObj);
    if (moving != nullptr)
      moving->synchronizeObserver();
  }
}

void
DynScene::shutdown()
{
  StaticScene::shutdown();
  dynObjs.clear();
}

// ***  SIMULATION STEP  *** //
// ************************* //
bool
DynScene::doSimStep()
{
  if (stepLoop.doStep())
    return stepLoop.retrieveOutput();
  return false;
}

bool
DynScene::doStep()
{
  bool updateFlag = false;
  size_t const n = numDynObjects();
  for (size_t i = 0; i < n; ++i) {
    updated[i] = dynObjs[i]->doStep();
    updateFlag |= updated[i];
  }
  return updateFlag;
}

void
DynScene::makeStepLoop(int const stepInterval)
{
  this->stepLoop =
    NonVoidStepLoop<bool>(stepInterval, [&]() -> bool { return doStep(); });
}
