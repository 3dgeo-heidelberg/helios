#include "FiberArrayBeamDeflector.h"
#include "maths/Directions.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractBeamDeflector>
FiberArrayBeamDeflector::clone()
{
  std::shared_ptr<AbstractBeamDeflector> fabd =
    std::make_shared<FiberArrayBeamDeflector>(
      FiberArrayBeamDeflector(cfg_device_scanAngleMax_rad,
                              cfg_device_scanFreqMax_Hz,
                              cfg_device_scanFreqMin_Hz,
                              cfg_device_numFibers));
  _clone(fabd);
  return fabd;
}
void
FiberArrayBeamDeflector::_clone(std::shared_ptr<AbstractBeamDeflector> abd)
{
  AbstractBeamDeflector::_clone(abd);
  FiberArrayBeamDeflector* fabd = (FiberArrayBeamDeflector*)abd.get();
  fabd->cfg_device_numFibers = cfg_device_numFibers;
  fabd->state_currentFiber = state_currentFiber;
}

// ***  M E T H O D S  *** //
// *********************** //
void
FiberArrayBeamDeflector::applySettings(
  std::shared_ptr<ScannerSettings> settings)
{
  AbstractBeamDeflector::applySettings(settings);

  state_currentBeamAngle_rad = -this->cfg_setting_scanAngle_rad;
  setNumFibers(cfg_device_numFibers);
}

void
FiberArrayBeamDeflector::setNumFibers(int numFibers)
{
  this->cfg_device_numFibers = numFibers;

  cached_angleBetweenPulses_rad =
    (this->cfg_setting_scanAngle_rad * 2) / cfg_device_numFibers;
}

void
FiberArrayBeamDeflector::doSimStep()
{
  state_currentBeamAngle_rad =
    -this->cfg_setting_scanAngle_rad +
    cached_angleBetweenPulses_rad * state_currentFiber;

  state_currentFiber++;
  if (state_currentFiber >= cfg_device_numFibers) {
    state_currentFiber = 0;
  }

  // Compute relative beam direction:
  this->cached_emitterRelativeAttitude =
    Rotation(Directions::right, state_currentBeamAngle_rad);
}
