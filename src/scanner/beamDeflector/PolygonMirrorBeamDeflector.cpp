#include "PolygonMirrorBeamDeflector.h"

#include "maths/Directions.h"
#include <iostream>
#include <logging.hpp>
#include <sstream>
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

  std::stringstream ss;
  ss << "Applying settings for PolygonMirrorBeamDeflector...\n";
  ss << "Vertical angle min/max "
     << MathConverter::radiansToDegrees(cfg_setting_verticalAngleMin_rad) << "/"
     << MathConverter::radiansToDegrees(cfg_setting_verticalAngleMax_rad)
     << " degrees";

  // if not set, use the ones from the scanAngleEffectiveMax or scanAngle
  // (whichever is lower)
  if (std::isnan(cfg_setting_verticalAngleMin_rad)) {
    cfg_setting_verticalAngleMin_rad =
      -1. *
      std::min(cfg_device_scanAngleEffectiveMax_rad, cfg_setting_scanAngle_rad);
    ss << "\n -- verticalAngleMin not set, using the value of "
       << MathConverter::radiansToDegrees(cfg_setting_verticalAngleMin_rad)
       << " degrees";
  }
  if (std::isnan(cfg_setting_verticalAngleMax_rad)) {
    cfg_setting_verticalAngleMax_rad =
      std::min(cfg_device_scanAngleEffectiveMax_rad, cfg_setting_scanAngle_rad);
    ss << "\n -- verticalAngleMax not set, using the value of "
       << MathConverter::radiansToDegrees(cfg_setting_verticalAngleMax_rad)
       << " degrees";
  }

  // ensure that the range between min and max vertical angle is not bigger than
  // the effective scan angle
  double verticalAngleRange_rad =
    cfg_setting_verticalAngleMax_rad - cfg_setting_verticalAngleMin_rad;
  if (verticalAngleRange_rad > cfg_device_scanAngleEffectiveMax_rad * 2) {
    // rase an error and exit
    std::stringstream ss;
    ss << "Error: The range between verticalAngleMin and verticalAngleMax ("
       << MathConverter::radiansToDegrees(verticalAngleRange_rad)
       << " degrees) cannot be bigger than the effective maximum scan angle "
          "(scanAngleEffectiveMax) of the device ("
       << MathConverter::radiansToDegrees(cfg_device_scanAngleEffectiveMax_rad *
                                          2)
       << " degrees). Please adjust the settings.";
    logging::ERR(ss.str());
    throw HeliosException(ss.str());
  }
  state_currentBeamAngle_rad = 0;
  logging::INFO(ss.str());

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
  // two conditions for the beam to return an echo:
  // 1) currentAngle > verticalAngleMin
  // 2) currentAngle <= verticalAngleMax

  return this->state_currentBeamAngle_rad >
           this->cfg_setting_verticalAngleMin_rad &&
         this->state_currentBeamAngle_rad <=
           this->cfg_setting_verticalAngleMax_rad;
}
