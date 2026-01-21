#include "RisleyBeamDeflector.h"

#include <memory>

#include <glm/glm.hpp>

#include "maths/Directions.h"

using Base = std::shared_ptr<AbstractBeamDeflector>;

// Construction/Cloning
Base
RisleyBeamDeflector::clone()
{
  Base ombd = std::make_shared<RisleyBeamDeflector>(prisms, refrIndex_air, incidentBeam, timeOffset);

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
  deltaT = 1.0 / settings->pulseFreq_Hz;
}

void
RisleyBeamDeflector::doSimStep()
{
  // Compute deflection at the current time. This allows the constructor or
  // XML-parsed 'timeOffset' to set the initial orientation (time) so the
  // very first simulated pulse is emitted at t = timeOffset (not
  // timeOffset + deltaT). After computing the deflection for the current
  // step we advance time by deltaT for the next call.

  // Start with incident beam
  glm::dvec3 beam = incidentBeam;

  for (const Prism& prism : prisms) {
    glm::dvec3 beamRefracted;
    if (!prism.refractPrism(beam, time, refrIndex_air, beamRefracted))
      return;

    beam = beamRefracted;
  }

  this->cached_emitterRelativeAttitude = Rotation(Directions::forward, beam);

  // Advance time after computing the deflection so that the first
  // computation uses the already-applied timeOffset (if any).
  time += deltaT;
}
