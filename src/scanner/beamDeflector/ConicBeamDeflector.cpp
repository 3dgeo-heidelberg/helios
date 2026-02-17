#include <helios/scanner/beamDeflector/ConicBeamDeflector.h>

#define _USE_MATH_DEFINES
#include "math.h"

#include <helios/maths/Directions.h>
// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractBeamDeflector>
ConicBeamDeflector::clone()
{
  std::shared_ptr<AbstractBeamDeflector> cbd =
    std::make_shared<ConicBeamDeflector>(
      ConicBeamDeflector(cfg_device_scanAngleMax_rad,
                         cfg_device_scanFreqMax_Hz,
                         cfg_device_scanFreqMin_Hz));
  _clone(cbd);
  return cbd;
}
void
ConicBeamDeflector::_clone(std::shared_ptr<AbstractBeamDeflector> abd)
{
  AbstractBeamDeflector::_clone(abd);
  ConicBeamDeflector* cbd = (ConicBeamDeflector*)abd.get();
  cbd->r1 = Rotation(r1);
}

// ***  M E T H O D S  *** //
// *********************** //
void
ConicBeamDeflector::applySettings(std::shared_ptr<ScannerSettings> settings)
{
  AbstractBeamDeflector::applySettings(settings);
  cached_angleBetweenPulses_rad =
    (double)(this->cfg_device_scanFreqMax_Hz * M_PI * 2) /
    settings->pulseFreq_Hz;
  r1 = Rotation(glm::dvec3(1, 0, 0), this->cfg_setting_scanAngle_rad);
}

void
ConicBeamDeflector::doSimStep()
{
  // ####### BEGIN Update mirror angle ########
  state_currentBeamAngle_rad += cached_angleBetweenPulses_rad;
  if (state_currentBeamAngle_rad >= M_PI * 2) {
    state_currentBeamAngle_rad = fmod(state_currentBeamAngle_rad, M_PI * 2);
  }
  // ####### END Update mirror angle ########
  Rotation r2 = Rotation(Directions::forward, state_currentBeamAngle_rad);
  this->cached_emitterRelativeAttitude = r2.applyTo(r1);
}
