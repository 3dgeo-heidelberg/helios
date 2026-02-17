#include <helios/scanner/detector/DynFullWaveformPulseRunnable.h>
#include <helios/scanner/detector/FullWaveformPulseRunnable.h>
#include <helios/scanner/detector/PulseTaskFactory.h>

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
std::shared_ptr<PulseTask>
PulseTaskFactory::buildFullWaveformPulseRunnable(
  ScanningPulseProcess const& spp,
  SimulatedPulse const& sp) const
{
  return std::make_shared<FullWaveformPulseRunnable>(spp.getScanner(), sp);
}

std::shared_ptr<PulseTask>
PulseTaskFactory::buildDynFullWaveformPulseRunnable(
  ScanningPulseProcess const& spp,
  SimulatedPulse const& sp) const
{
  return std::make_shared<DynFullWaveformPulseRunnable>(
    scene.getRaycaster()->makeTemporalClone(), spp.getScanner(), sp);
}

void
PulseTaskFactory::configureBuildMethod()
{
  if (scene.hasMovingObjects()) {
    _build = [&](ScanningPulseProcess const& spp,
                 SimulatedPulse const& sp) -> std::shared_ptr<PulseTask> {
      return buildDynFullWaveformPulseRunnable(spp, sp);
    };
  } else {
    _build = [&](ScanningPulseProcess const& spp,
                 SimulatedPulse const& sp) -> std::shared_ptr<PulseTask> {
      return buildFullWaveformPulseRunnable(spp, sp);
    };
  }
}
