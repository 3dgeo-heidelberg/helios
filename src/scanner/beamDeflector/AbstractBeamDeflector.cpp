#include "AbstractBeamDeflector.h"

#include <memory>
#include <sstream>
#include <string>

#include "MathConverter.h"
#include <logging.hpp>

#define _USE_MATH_DEFINES

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
void
AbstractBeamDeflector::_clone(std::shared_ptr<AbstractBeamDeflector> abd)
{
  this->cfg_device_scanFreqMax_Hz = abd->cfg_device_scanFreqMax_Hz;
  this->cfg_device_scanFreqMin_Hz = abd->cfg_device_scanFreqMin_Hz;
  this->cfg_device_scanAngleMax_rad = abd->cfg_device_scanAngleMax_rad;
  this->cfg_setting_scanFreq_Hz = abd->cfg_setting_scanFreq_Hz;
  this->cfg_setting_scanAngle_rad = abd->cfg_setting_scanAngle_rad;
  this->cfg_setting_verticalAngleMin_rad =
    abd->cfg_setting_verticalAngleMin_rad;
  this->cfg_setting_verticalAngleMax_rad =
    abd->cfg_setting_verticalAngleMax_rad;
  this->state_currentBeamAngle_rad = abd->state_currentBeamAngle_rad;
  this->state_angleDiff_rad = abd->state_angleDiff_rad;
  this->cached_angleBetweenPulses_rad = abd->cached_angleBetweenPulses_rad;
  this->cached_emitterRelativeAttitude = abd->cached_emitterRelativeAttitude;
}

// ***  M E T H O D S  *** //
// *********************** //
void
AbstractBeamDeflector::applySettings(std::shared_ptr<ScannerSettings> settings)
{
  setScanAngle_rad(settings->scanAngle_rad);
  setScanFreq_Hz(settings->scanFreq_Hz);
  cfg_setting_verticalAngleMin_rad = settings->verticalAngleMin_rad;
  cfg_setting_verticalAngleMax_rad = settings->verticalAngleMax_rad;
  double angleMax = cfg_setting_scanAngle_rad;
  double angleMin = -cfg_setting_scanAngle_rad;
  if (angleMax == 0.0) {
    angleMax = cfg_setting_verticalAngleMax_rad;
    angleMin = cfg_setting_verticalAngleMin_rad;
  } else {
    cfg_setting_verticalAngleMin_rad = angleMin;
    cfg_setting_verticalAngleMax_rad = angleMax;
  }
  state_angleDiff_rad = angleMax - angleMin;
  if (cfg_setting_scanAngle_rad == 0.0) {
    cfg_setting_scanAngle_rad = state_angleDiff_rad / 2.0;
  }
  cached_angleBetweenPulses_rad =
    (double)(this->cfg_setting_scanFreq_Hz * state_angleDiff_rad) /
    settings->pulseFreq_Hz;
}

Rotation
AbstractBeamDeflector::getEmitterRelativeAttitude()
{
  return this->cached_emitterRelativeAttitude;
}

void
AbstractBeamDeflector::setScanAngle_rad(double scanAngle_rad)
{
  if (scanAngle_rad < 0) {
    scanAngle_rad = 0;
  } else if (scanAngle_rad > cfg_device_scanAngleMax_rad) {
    scanAngle_rad = cfg_device_scanAngleMax_rad;
  }
  // scanAngle of 0 would mean that no points are recorded - set to scanAngleMax
  // instead
  if (scanAngle_rad == 0) {
    scanAngle_rad = cfg_device_scanAngleMax_rad;
  }
  this->cfg_setting_scanAngle_rad = scanAngle_rad;
  std::stringstream ss;
  ss << "Scan angle set to "
     << MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad);
  logging::INFO(ss.str());
}

bool
AbstractBeamDeflector::lastPulseLeftDevice()
{
  return true;
}

void
AbstractBeamDeflector::restartDeflector()
{
  state_currentBeamAngle_rad = 0;
}

void
AbstractBeamDeflector::setScanFreq_Hz(double scanFreq_hz)
{
  cfg_setting_scanFreq_Hz = scanFreq_hz;
}
