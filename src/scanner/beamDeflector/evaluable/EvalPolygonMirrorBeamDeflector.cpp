#include <maths/Directions.h>
#include <scanner/beamDeflector/evaluable/EvalPolygonMirrorBeamDeflector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractBeamDeflector>
EvalPolygonMirrorBeamDeflector::clone()
{
  std::shared_ptr<AbstractBeamDeflector> epmbd =
    std::make_shared<EvalPolygonMirrorBeamDeflector>(
      cfg_device_scanFreqMax_Hz,
      cfg_device_scanFreqMin_Hz,
      cfg_device_scanAngleMax_rad,
      cfg_device_scanAngleEffectiveMax_rad);
  _clone(epmbd);
  return epmbd;
}
void
EvalPolygonMirrorBeamDeflector::_clone(
  std::shared_ptr<AbstractBeamDeflector> abd)
{
  PolygonMirrorBeamDeflector::_clone(abd);
  EvalPolygonMirrorBeamDeflector* epmbd =
    (EvalPolygonMirrorBeamDeflector*)abd.get();
  epmbd->state_currentExactBeamAngle_rad =
    this->state_currentExactBeamAngle_rad;
}

// ***  M E T H O D S  *** //
// *********************** //
void
EvalPolygonMirrorBeamDeflector::applySettings(
  std::shared_ptr<ScannerSettings> settings)
{
  PolygonMirrorBeamDeflector::applySettings(settings);
  state_currentExactBeamAngle_rad = state_currentBeamAngle_rad;
}
void
EvalPolygonMirrorBeamDeflector::doSimStep()
{
  // Update beam angle
  state_currentExactBeamAngle_rad += cached_angleBetweenPulses_rad;
  state_currentBeamAngle_rad =
    state_currentExactBeamAngle_rad +
    vertAngErrExpr->eval(state_currentExactBeamAngle_rad);

  // Fit beam angle to domain
  if (state_currentExactBeamAngle_rad >= cfg_device_scanAngleMax_rad) {
    state_currentExactBeamAngle_rad = -cfg_device_scanAngleMax_rad;
  }
  if (state_currentBeamAngle_rad >= cfg_device_scanAngleMax_rad) {
    state_currentBeamAngle_rad = -cfg_device_scanAngleMax_rad;
  }

  // Rotate to current position:
  this->cached_exactEmitterRelativeAttitude =
    Rotation(Directions::right, state_currentExactBeamAngle_rad);
  this->cached_emitterRelativeAttitude =
    Rotation(Directions::right, state_currentBeamAngle_rad);
}
