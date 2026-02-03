#include "PolygonMirrorBeamDeflector.h"

#include "maths/Directions.h"
#define _USE_MATH_DEFINES
#include "MathConverter.h"
#include <math.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractBeamDeflector>
PolygonMirrorBeamDeflector::clone()
{
  std::shared_ptr<AbstractBeamDeflector> pmbd =
    std::make_shared<PolygonMirrorBeamDeflector>(
      cfg_device_scanFreqMax_Hz,
      cfg_device_scanFreqMin_Hz,
      cfg_device_scanAngleMax_rad,
      cfg_device_scanAngleEffectiveMax_rad);
  _clone(pmbd);
  return pmbd;
};
void
PolygonMirrorBeamDeflector::_clone(std::shared_ptr<AbstractBeamDeflector> abd)
{
  AbstractBeamDeflector::_clone(abd);
  PolygonMirrorBeamDeflector* pmbd = (PolygonMirrorBeamDeflector*)abd.get();
  pmbd->cfg_device_scanAngleEffective_rad =
    this->cfg_device_scanAngleEffective_rad;
  pmbd->cfg_device_scanAngleEffectiveMax_rad =
    this->cfg_device_scanAngleEffectiveMax_rad;
};

// ***  M E T H O D S  *** //
// *********************** //

void
PolygonMirrorBeamDeflector::applySettings(
  std::shared_ptr<ScannerSettings> settings)
{
  setScanAngle_rad(settings->scanAngle_rad);
  setScanFreq_Hz(settings->scanFreq_Hz);

  // get the verticalAngle settings (suggested for TLS)
  cfg_setting_verticalAngleMin_rad = settings->verticalAngleMin_rad;
  cfg_setting_verticalAngleMax_rad = settings->verticalAngleMax_rad;

  // if not set, use the ones from the scanAngleEffectiveMax or scanAngle
  // (whichever is lower)
  if (std::isnan(cfg_setting_verticalAngleMin_rad)) {
    cfg_setting_verticalAngleMin_rad =
      -1. *
      std::min(cfg_device_scanAngleEffectiveMax_rad, cfg_setting_scanAngle_rad);
  }
  if (std::isnan(cfg_setting_verticalAngleMax_rad)) {
    cfg_setting_verticalAngleMax_rad =
      std::min(cfg_device_scanAngleEffectiveMax_rad, cfg_setting_scanAngle_rad);
  }
  state_currentBeamAngle_rad = 0;

  // For calculating the spacing between subsequent shots:
  double const angleMax = cfg_device_scanAngleMax_rad;
  double const angleMin = -cfg_device_scanAngleMax_rad;

  state_angleDiff_rad = angleMax - angleMin;
  cached_angleBetweenPulses_rad =
    (double)(this->cfg_setting_scanFreq_Hz * state_angleDiff_rad) /
    settings->pulseFreq_Hz;
}

void
PolygonMirrorBeamDeflector::doSimStep()
{
  // Update beam angle:
  state_currentBeamAngle_rad += cached_angleBetweenPulses_rad;

  if (state_currentBeamAngle_rad >= cfg_device_scanAngleMax_rad) {
    state_currentBeamAngle_rad = -cfg_device_scanAngleMax_rad;
  }

  // Rotate to current position:
  this->cached_emitterRelativeAttitude =
    Rotation(Directions::right, state_currentBeamAngle_rad);
}

bool
PolygonMirrorBeamDeflector::lastPulseLeftDevice()
{
  // four conditions for the beam to return an echo:
  // 1) abs(currentAngle) <= scanAngleEffectiveMax
  // 2) abs(currentAngle) <= scanAngle (if it is set to something smaller)
  // 3) currentAngle > verticalAngleMin
  // 4) currentAngle <= verticalAngleMax

  return std::fabs(this->state_currentBeamAngle_rad) <=
           this->cfg_device_scanAngleEffectiveMax_rad &&
         std::fabs(this->state_currentBeamAngle_rad) <=
           this->cfg_setting_scanAngle_rad &&
         this->state_currentBeamAngle_rad >
           this->cfg_setting_verticalAngleMin_rad &&
         this->state_currentBeamAngle_rad <=
           this->cfg_setting_verticalAngleMax_rad;
}
