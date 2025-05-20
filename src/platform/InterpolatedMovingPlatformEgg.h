#pragma once

#include <assetloading/EggAsset.h>
#include <platform/InterpolatedMovingPlatform.h>
#include <sim/comps/SimulationStepLoop.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Egg class that can hatch to a full InterpolatedMovingPlatform class
 *
 * It is mainly used to facilitate asset loading of InterpolatedMovingPlatform
 * @see InterpolatedMovingPlatform
 */
class InterpolatedMovingPlatformEgg
  : public EggAsset<InterpolatedMovingPlatform, SimulationStepLoop&>
  , public MovingPlatform
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  InterpolatedMovingPlatform::InterpolationScope scope =
    InterpolatedMovingPlatform::InterpolationScope::POSITION_AND_ATTITUDE;
  InterpolatedMovingPlatform::RotationSpec rotspec =
    InterpolatedMovingPlatform::RotationSpec::CANONICAL;
  std::shared_ptr<fluxionum::TemporalDesignMatrix<double, double>> tdm;
  std::shared_ptr<fluxionum::DiffDesignMatrix<double, double>> ddm;
  bool syncGPSTime; // If true, set start GPS time to first time from data
  double startTime; // To be used when syncGPSTime is true and also to
                    // translate to simulation time s.t. -startTime
                    // It is given in seconds, not nanoseconds

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  InterpolatedMovingPlatformEgg()
    : EggAsset()
    , scope(
        InterpolatedMovingPlatform::InterpolationScope::POSITION_AND_ATTITUDE)
    , tdm(nullptr)
    , ddm(nullptr)
    , syncGPSTime(false)
    , startTime(0)
  {
  }
  ~InterpolatedMovingPlatformEgg() override = default;

  // ***  EGG METHODS  *** //
  // ********************* //
  InterpolatedMovingPlatform hatch(SimulationStepLoop& stepLoop) override
  {
    InterpolatedMovingPlatform imp(
      stepLoop, *tdm, *ddm, scope, syncGPSTime, startTime, rotspec);
    fulfillPlatform(imp);
    return imp;
  }
  std::shared_ptr<InterpolatedMovingPlatform> smartHatch(
    SimulationStepLoop& stepLoop) override
  {
    std::shared_ptr<InterpolatedMovingPlatform> imp =
      std::make_shared<InterpolatedMovingPlatform>(
        stepLoop, *tdm, *ddm, scope, syncGPSTime, startTime, rotspec);
    fulfillPlatform(*imp);
    return imp;
  }

protected:
  // ***  INNER UTIL METHODS  *** //
  // **************************** //
  /**
   * @brief Fulfill given InterpolatedMovingPlatform with data from the egg
   * @param imp InterpolatedMovingPlatform to be fulfilled from egg
   */
  void fulfillPlatform(InterpolatedMovingPlatform& imp)
  {
    // Platform attributes
    imp.cfg_device_relativeMountPosition = cfg_device_relativeMountPosition;
    imp.cfg_device_relativeMountAttitude = cfg_device_relativeMountAttitude;
    imp.lastCheckZ = lastCheckZ;
    imp.lastGroundCheck = lastGroundCheck;
    imp.scene = scene;
    imp.positionXNoiseSource = positionXNoiseSource;
    imp.positionYNoiseSource = positionYNoiseSource;
    imp.positionZNoiseSource = positionZNoiseSource;
    imp.attitudeXNoiseSource = attitudeXNoiseSource;
    imp.attitudeYNoiseSource = attitudeYNoiseSource;
    imp.attitudeZNoiseSource = attitudeZNoiseSource;
    imp.dmax = dmax;
    imp.prevWrittenPos = prevWrittenPos;
    imp.cfg_settings_movePerSec_m = cfg_settings_movePerSec_m;
    imp.originWaypoint = originWaypoint;
    imp.targetWaypoint = targetWaypoint;
    imp.nextWaypoint = nextWaypoint;
    imp.onGround = onGround;
    imp.stopAndTurn = stopAndTurn;
    imp.smoothTurn = smoothTurn;
    imp.slowdownEnabled = slowdownEnabled;
    imp.position = position;
    imp.attitude = attitude;
    imp.mSetOrientationOnLegInit = mSetOrientationOnLegInit;
    imp.writeNextTrajectory = writeNextTrajectory;
    // Platform cached attributes
    imp.cached_absoluteMountPosition = cached_absoluteMountPosition;
    imp.cached_absoluteMountAttitude = cached_absoluteMountAttitude;
    imp.cached_dir_current = cached_dir_current;
    imp.cached_dir_current_xy = cached_dir_current_xy;
    imp.cached_vectorToTarget = cached_vectorToTarget;
    imp.cached_vectorToTarget_xy = cached_vectorToTarget_xy;
    imp.cached_distanceToTarget_xy = cached_distanceToTarget_xy;
    imp.cached_originToTargetDir_xy = cached_originToTargetDir_xy;
    imp.cached_targetToNextDir_xy = cached_targetToNextDir_xy;
    imp.cached_endTargetAngle_xy = cached_endTargetAngle_xy;
    imp.cached_currentAngle_xy = cached_currentAngle_xy;
    imp.cached_originToTargetAngle_xy = cached_originToTargetAngle_xy;
    imp.cached_targetToNextAngle_xy = cached_targetToNextAngle_xy;
    // Moving platform attributes
    imp.setVelocity(getVelocity());
  }

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @see EggAsset::isEgg
   */
  bool isEgg() const override
  {
    return EggAsset<InterpolatedMovingPlatform, SimulationStepLoop&>::isEgg();
  }
};
