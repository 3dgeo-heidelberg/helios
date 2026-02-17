#include <helios/scanner/EvalScannerHead.h>

#define _USE_MATH_DEFINES
#include <helios/maths/MathConstants.h>
#include <helios/maths/TrigoTricks.h>
#include <math.h>

// ***  M E T H O D S  *** //
// *********************** //
void
EvalScannerHead::doSimStep(double pulseFreq_Hz)
{
  if (cfg_setting_rotatePerSec_rad != 0) {
    setCurrentRotateAngle_rad(state_currentExactRotateAngle_rad +
                              cfg_setting_rotatePerSec_rad / pulseFreq_Hz);
  }
}

bool
EvalScannerHead::rotateCompleted()
{
  bool result = false;

  if (cfg_setting_rotatePerSec_rad < 0) {
    result = state_currentExactRotateAngle_rad <= cfg_setting_rotateStop_rad;
  } else {
    result = state_currentExactRotateAngle_rad >= cfg_setting_rotateStop_rad;
  }

  return result;
}

// ***  GETTERS and SETTERS  *** //
// ***************************** //
Rotation
EvalScannerHead::getExactMountRelativeAttitude()
{
  return this->cached_exactMountRelativeAttitude;
}

void
EvalScannerHead::setCurrentRotateAngle_rad(double angle_rad)
{
  if (angle_rad == state_currentExactRotateAngle_rad)
    return;
  // Configure exact rotate angle state
  state_currentExactRotateAngle_rad = angle_rad;
  cached_exactMountRelativeAttitude = Rotation(
    cfg_device_rotateAxis, fmod(state_currentExactRotateAngle_rad, PI_2));
  // WARNING! Error expression below requires deflectorAngle does not
  // lead to sin(deflectorAngle) approx 0
  double const theta =
    TrigoTricks::clipZeroSinRadians(*deflectorAngle, // Vertical angle (rad)
                                    zeroSinThreshold_rad);
  state_currentRotateAngle_rad =
    state_currentExactRotateAngle_rad + horizAngErrExpr->eval(theta);
  cached_mountRelativeAttitude =
    Rotation(cfg_device_rotateAxis, fmod(state_currentRotateAngle_rad, PI_2));
}
