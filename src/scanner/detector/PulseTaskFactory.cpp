#include <DynFullWaveformPulseRunnable.h>
#include <FullWaveformPulseRunnable.h>
#include <PulseTaskFactory.h>

using std::dynamic_pointer_cast;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
PulseTaskFactory::PulseTaskFactory(Scene& scene)
  : scene(scene)
{
  configureBuildMethod();
}

// ***  FACTORY METHODS  *** //
// ************************* //
shared_ptr<PulseTask>
PulseTaskFactory::buildFullWaveformPulseRunnable(
  ScanningPulseProcess const& spp,
  SimulatedPulse const& sp) const
{
  return make_shared<FullWaveformPulseRunnable>(spp.getScanner(), sp);
}

shared_ptr<PulseTask>
PulseTaskFactory::buildDynFullWaveformPulseRunnable(
  ScanningPulseProcess const& spp,
  SimulatedPulse const& sp) const
{
  return make_shared<DynFullWaveformPulseRunnable>(
    scene.getRaycaster()->makeTemporalClone(), spp.getScanner(), sp);
}

void
PulseTaskFactory::configureBuildMethod()
{
  if (scene.hasMovingObjects()) {
    _build = [&](ScanningPulseProcess const& spp,
                 SimulatedPulse const& sp) -> shared_ptr<PulseTask> {
      return buildDynFullWaveformPulseRunnable(spp, sp);
    };
  } else {
    _build = [&](ScanningPulseProcess const& spp,
                 SimulatedPulse const& sp) -> shared_ptr<PulseTask> {
      return buildFullWaveformPulseRunnable(spp, sp);
    };
  }
}
