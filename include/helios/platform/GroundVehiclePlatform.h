#pragma once

#include <helios/platform/SimplePhysicsPlatform.h>

#define _USE_MATH_DEFINES
#include "math.h"

/**
 * @brief Class representing a ground vehicle platform
 */
class GroundVehiclePlatform : public SimplePhysicsPlatform
{

private:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Max force for vehicle engine
   */
  double mEngineForceMax = 0.2;
  /**
   * @brief Current force of vehicle engine
   */
  double mEngineForceCurrent = 0;
  /**
   * @brief Target force for vehicle engine
   */
  double mEngineForceTarget = 0;
  /**
   * @brief Turning threshold in radians
   */
  double mComplexTurnThreshold_rad = M_PI / 5;
  /**
   * @brief Turn mode specification
   *
   * <ul>
   *  <li>0 for normal forward driving and wide angle curves</li>
   *  <li>1 for first stage of two-step turn for narrow angle curves</li>
   *  <li>2 for second stage of two-step turn for narrow angle curves</li>
   * </ul>
   */
  int mTurnMode = 0;
  /**
   * @brief Temporary way point to assist ground vehicle movement
   */
  glm::dvec3 mTempWaypoint;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for ground vehicle platform
   */
  GroundVehiclePlatform()
    : SimplePhysicsPlatform()
  {
    // Disable gravity:
    mCfg_g_accel = glm::dvec3(0, 0, 0);
  }
  std::shared_ptr<Platform> clone() override;
  void _clone(std::shared_ptr<Platform> p) override;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @see SimplePhyisicsPlatform::doControlStep
   */
  void doControlStep(int simFrequency_hz) override;
  /**
   * @see Platform::setDestination
   * @param dest
   */
  void setDestination(glm::dvec3 dest) override;
  /**
   * @see Platform::prepareSimulation
   */
  void prepareSimulation(int simFrequency_hz) override;
};
