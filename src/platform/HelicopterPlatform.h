#pragma once

#include "SimplePhysicsPlatform.h"

#include <glm/glm.hpp>

#include "maths/Rotation.h"

/**
 * @brief Class representing a helicopter platform
 */
class HelicopterPlatform : public SimplePhysicsPlatform
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Distance threshold to start slowdown process when not in
   * smooth turn mode
   */
  double cfg_slowdown_dist_xy = 5.0;
  /**
   * @brief Slowdown step magnitude
   */
  double cfg_slowdown_magnitude = 2.0;
  /**
   * @brief Speedup step magnitude
   */
  double cfg_speedup_magnitude = 2.0;
  /**
   * @brief Max engine force over XY plane
   */
  double ef_xy_max = 0.1;
  /**
   * @brief Helicopter yaw angle
   */
  double yaw = 0.0;
  /**
   * @brief Helicopter roll angle
   */
  double roll = 0.0;
  /**
   * @brief Helicopter pitch angle
   */
  double pitch = 0.0;
  /**
   * @brief Helicopter rotation sign coming from previous simulation step
   */
  double lastSign = 1.0;
  /**
   * @brief Base pitch angle for the helicopter
   */
  double cfg_pitch_base = -0.087;
  /**
   * @brief Yaw rotation speed (units per second)
   */
  double cfg_yaw_speed = 1.5;
  /**
   * @brief Roll rotation speed (units per second)
   */
  double cfg_roll_speed = 0.5;
  /**
   * @brief Pitch rotation speed (units per second)
   */
  double cfg_pitch_speed = 1.5;
  /**
   * @brief Maximum boundary for roll angle with respect to base roll
   */
  double cfg_max_roll_offset = 0.45;
  /**
   * @brief Maximum boundary for pitch angle with respect to base pitch
   */
  double cfg_max_pitch_offset = 0.61;
  /**
   * @brief Maximum boundary for pitch angle (radians)
   */
  double cfg_max_pitch = cfg_pitch_base + cfg_max_pitch_offset;
  /**
   * @brief Minimum boundary for pitch angle (radians)
   */
  double cfg_min_pitch = cfg_pitch_base - cfg_max_pitch_offset;
  /**
   * @brief Helicopter slowdown factor
   *
   * \f[
   *  \textrm{slowdownFactor} =
   *      1 - \frac{\textrm{slowdownMagnitude}}{\textrm{simFrequency}}
   * \f]
   */
  double cfg_slowdownFactor;
  /**
   * @brief Helicopter speedup factor
   *
   * \f[
   *  \textrm{speedupFactor} =
   *      1 + \frac{\textrm{speedupMagnitude}}{\textrm{simFrequency}}
   * \f]
   */
  double cfg_speedupFactor;
  /**
   * @brief Pitch step magnitude
   *
   * \f[
   *  \textrm{pitchStepMagnitude} =
   *      \frac{\textrm{pitchSpeed}}{\textrm{simFrequency}}
   * \f]
   */
  double cfg_pitchStepMagnitude = 0;
  /**
   * @brief Roll step magnitude
   *
   * \f[
   *  \textrm{rollStepMagnitude} =
   *      \frac{\textrm{rollSpeed}}{\textrm{simFrequency}}
   * \f]
   */
  double cfg_rollStepMagnitude = 0;
  /**
   * @brief Yaw step magnitude
   *
   * \f[
   *  \textrm{yawnStepMagnitude} =
   *      \frac{\textrm{yawSpeed}}{\textrm{simFrequency}}
   * \f]
   */
  double cfg_yawStepMagnitude = 0;
  /**
   * @brief Helicopter alignment threshold. A helicopter will be considered
   * as aligned when the difference between target yaw and current yaw
   * does not exceed this threshold
   */
  double cfg_alignmentThreshold = 0.001;

  /**
   * @brief Helicopter speed vector over xy plane
   */
  glm::dvec3 speed_xy = glm::dvec3(0, 0, 0);
  /**
   * @brief Helicopter speed vector over xy plane from previous simulation
   * step
   */
  glm::dvec3 lastSpeed_xy = glm::dvec3(0, 0, 0);

  /**
   * @brief Rotation instance to assist helicopter rotation computation
   */
  Rotation r = Rotation(glm::dvec3(1, 0, 0), 0);
  /**
   * @brief Directional attitude over XY plane for HelicopterPlatform
   * @see HelicopterPlatform::getDirectionalAttitude
   */
  Rotation dirAttitudeXY = Rotation(Directions::up, 0);

  // ***  CACHE ATTRIBUTES  *** //
  // ************************** //
  /**
   * @brief Count of turn iterations
   */
  int cache_turnIterations = 0;
  /**
   * @brief Flag specifying if helicopter is currently turning (true)
   * or not (false)
   */
  bool cache_turning = false;
  /**
   * @biref Flag specifying if helicopter is currently aligning (true)
   * or not (false)
   */
  bool cache_aligning = false;
  /**
   * @brief XY distance threshold to handle turning computation
   */
  double cache_xyDistanceThreshold;
  /**
   * @brief Flag specifying if helicopter speed-up stage has finished (true)
   * or not (false)
   */
  bool cache_speedUpFinished = false;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default helicopter platform constructor
   */
  HelicopterPlatform() = default;
  std::shared_ptr<Platform> clone() override;
  void _clone(std::shared_ptr<Platform> p) override;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @see Platform::prepareSimulation
   */
  void prepareSimulation(int simFrequency_hz) override;
  /**
   * @see Platform::initLegManual
   */
  void initLegManual() override;
  /**
   * @see Platform::initLeg
   */
  void initLeg() override;
  /**
   * @see Platform::waypointReached
   */
  bool waypointReached() override;
  /**
   * @see Platform::updateStaticCache
   */
  void updateStaticCache() override;
  /**
   * @brief Compute the distance threshold with respect to expected turn
   */
  void computeTurnDistanceThreshold();
  /**
   * @brief Compute the slowdown distance. This is necessary when using the
   *  slowdown mode with no smooth turn. Otherwise, the helicopter will stop
   *  before arriving to destination.
   *
   * @see HelicopterPlatform::cfg_slowdown_dist_xy
   */
  void computeNonSmoothSlowdownDist();

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @see Platform::setHeadingRad
   */
  void setHeadingRad(double rad) override
  {
    yaw = stopAndTurn ? rad - M_PI : rad;
  }
  /**
   * @see Platform::getHeadingRad
   */
  double getHeadingRad() override { return yaw; }
  /**
   * @brief Obtain xy speed vector by reference
   * @return Reference to xy speed vector
   */
  glm::dvec3& getSpeedXyByReference() { return speed_xy; }
  /**
   * @brief Obtain rotation assistance r by reference
   * @return Reference to rotation assistance r
   */
  Rotation& getRotationByReference() { return r; }

  /**
   * @brief Directional attitude for HelicopterPlatform considers only the
   * XY direction
   * @see Platform::getDirectionalAttitude
   */
  Rotation getDirectionalAttitude() override { return this->dirAttitudeXY; }
  /**
   * @see Platform::getCurrentDirection
   */
  glm::dvec3 getCurrentDirection() override;
  /**
   * @see Platform::canStopAndTurn
   */
  bool canStopAndTurn() const override { return true; }

  // ***  CONTROL STEP  *** //
  // ********************** //
  void doControlStep(int simFrequency_hz) override;
  /**
   * @brief Compute the lift/sink rate
   * @return Ascend/descend speed
   */
  double computeLiftSinkRate();
  /**
   * @brief Compute the speed through XY and its impact on pitch
   */
  void computeXYSpeed(int simFrequency_hz);
  /**
   * @brief Compute the engine force
   */
  void computeEngineForce(double zForceTarget);
  /**
   * @brief Compute rotation angles (roll, pitch, yaw)
   * @param[in] simFrequency_hz Simulation frequency (in hz)
   * @see HelicopterPlatform::rotate
   */
  void computeRotationAngles(int simFrequency_hz);
  /**
   * @brief Compute the rotation angles for alignment process
   *
   * The alignment process is necessary to align platform orientation
   * at the beginning of a new leg when the turning process at the ending
   * of previous leg had changed it.
   */
  void computeAlignmentAngles();
  /**
   * @brief Compute the rotation angles for turning process
   *
   * The turning process changes platform orientation at the end of current
   * leg so the platform faces the next target waypoint
   */
  void computeTurningAngles();
  /**
   * @brief Rotate helicopter
   * @param[in] roll Rotation angle for \f$(y,z)\f$ over x-axis
   * @param[in] pitch Rotation angle for \f$(x,z)\f$ over y-axis
   * @param[in] yaw Rotation angle for \f$(x,y)\f$ over z-axis
   */
  void rotate(double roll, double pitch, double yaw);
  /**
   * @brief Determine remaining iterations and start turning mode if
   * necessary
   */
  void handleRoute(int simFrequency_hz);
  /**
   * @brief Compute speed magnitude after a slowdown step
   * @param[in] speedMagnitude The speed vector magnitude/norm
   * @return Speed magnitude after slowdown
   */
  inline double computeSlowdownStep(double const speedMagnitude) const
  {
    return cfg_slowdownFactor * speedMagnitude;
  }
  /**
   * @brief Compute speed magnitude after a speedup step
   * @param[in] speedMagnitude the speed vector magnitude/norm
   * @return Speed magnitude after speedup
   */
  inline double computeSpeedupStep(double const speedMagnitude) const
  {
    return cfg_speedupFactor * speedMagnitude;
  }
};
