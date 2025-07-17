#include "RisleyBeamDeflector.h"

#include <iostream>
#include <sstream>
#include <string>
using namespace std;

#include <glm/glm.hpp>
#define _USE_MATH_DEFINES
#include <logging.hpp>
#include <math.h>

#include "maths/Directions.h"
#include "maths/MathConverter.h"

using Base = std::shared_ptr<AbstractBeamDeflector>;

// Construction/Cloning
Base
RisleyBeamDeflector::clone()
{
  Base ombd = std::make_shared<RisleyBeamDeflector>(
    cfg_device_scanAngleMax_rad, prisms, refrIndex_air);

  _clone(ombd);
  return ombd;
}
void
RisleyBeamDeflector::_clone(std::shared_ptr<AbstractBeamDeflector> abd)
{
  AbstractBeamDeflector::_clone(abd);
  RisleyBeamDeflector* ombd = (RisleyBeamDeflector*)abd.get();
}

void
RisleyBeamDeflector::applySettings(std::shared_ptr<ScannerSettings> settings)
{
  AbstractBeamDeflector::applySettings(settings);
  cached_angleBetweenPulses_rad =
    (double)(this->cfg_setting_scanFreq_Hz * this->cfg_setting_scanAngle_rad *
             4) /
    settings->pulseFreq_Hz;
  scanAngle = this->cfg_setting_scanAngle_rad;
  deltaT = 1.0 / settings->pulseFreq_Hz;
}

void
RisleyBeamDeflector::doSimStep()
{
  // time integration
  time += deltaT;

  // Start with incident beam
  glm::dvec3 beam = incidentBeam;

  for (const Prism& prism : prisms) {
    glm::dvec3 beamRefracted;
    if (!prism.refractPrism(beam, time, refrIndex_air, beamRefracted))
      return;

    beam = beamRefracted;
  }

  this->cached_emitterRelativeAttitude = Rotation(Directions::forward, beam);
}

void
RisleyBeamDeflector::setScanAngle_rad(double scanAngle_rad)
{
  double scanAngle_deg = MathConverter::radiansToDegrees(scanAngle_rad);

  // Max. scan angle is limited by scan product:
  /*if (scanAngle_deg * this->cfg_setting_scanFreq_Hz >
  this->cfg_device_scanProduct) { logging::WARN( "ERROR: Requested scan angle
  exceeds device limitations " "as defined by scan product. " "Will set it to
  maximal possible value."
  );
          scanAngle_deg = ((double) this->cfg_device_scanProduct) /
  this->cfg_setting_scanFreq_Hz;
  }*/

  this->cfg_setting_scanAngle_rad =
    MathConverter::degreesToRadians(scanAngle_deg);
  stringstream ss;
  ss << "Scan angle set to " << scanAngle_deg << " degrees.";
  logging::INFO(ss.str());
}

// This setter method should not be used for this scanner.

void
RisleyBeamDeflector::setScanFreq_Hz(double scanFreq_Hz)
{
  // Max. scan frequency is limited by scan product:
  // if( MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad) *
  //    scanFreq_Hz > this->cfg_device_scanProduct
  //   ){
  //	logging::WARN(
  //	    "ERROR: Requested scan frequency exceeds device limitations "
  //           "as defined by scan product. "
  //           "Will set it to maximal possible value."
  //       );
  //	scanFreq_Hz = ((double) this->cfg_device_scanProduct) /
  //	    MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad);
  //}
  this->cfg_setting_scanFreq_Hz = scanFreq_Hz;
  stringstream ss;
  ss << "Scan frequency set to " << this->cfg_setting_scanFreq_Hz << " Hz.";
  logging::INFO(ss.str());
}
