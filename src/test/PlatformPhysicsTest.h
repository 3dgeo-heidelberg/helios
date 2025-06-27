#pragma once

#include "BaseTest.h"
#include <HelicopterPlatform.h>
#include <MathConstants.h>

namespace HeliosTests {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Platform physics test
 */
class PlatformPhysicsTest : public BaseTest
{
public:
  /**
   * @brief Decimal precision for validation purposes
   */
  double eps = 0.0001; // Decimal precision for validation purposes

  // ***  CONSTRUCTOR  *** //
  // ********************* //
  /**
   * @brief Platform physics test constructor
   */
  PlatformPhysicsTest()
    : BaseTest("Platform physics test")
  {
  }
  virtual ~PlatformPhysicsTest() = default;

  // ***  R U N  *** //
  // *************** //
  /**
   * @see BaseTest::run
   */
  bool run() override;

  // ***  SUB-TESTS  *** //
  // ******************* //
  /**
   * @brief Roll only rotations test
   * @return True if passed, false otherwise
   */
  bool testRollOnlyRotations();
  /**
   * @brief Pitch only rotations test
   * @return True if passed, false otherwise
   */
  bool testPitchOnlyRotations();
  /**
   * @brief Yaw only rotations test
   * @return True if passed, false otherwise
   */
  bool testYawOnlyRotations();
  /**
   * @brief Roll and pitch rotations test
   * @return True if passed, false otherwise
   */
  bool testRollPitchRotations();
  /**
   * @brief Roll yaw rotations test
   * @return True if passed, false otherwise
   */
  bool testRollYawRotations();
  /**
   * @brief Pitch yaw rotations test
   * @return True if passed, false otherwise
   */
  bool testPitchYawRotations();
  /**
   * @brief Roll, pitch and yaw rotations test
   * @return True if passed, false otherwise
   */
  bool testRollPitchYawRotations();
};

// ***  R U N  *** //
// *************** //
bool
PlatformPhysicsTest::run()
{
  if (!testRollOnlyRotations())
    return false;
  if (!testPitchOnlyRotations())
    return false;
  if (!testYawOnlyRotations())
    return false;
  if (!testRollPitchRotations())
    return false;
  if (!testRollYawRotations())
    return false;
  if (!testPitchYawRotations())
    return false;
  if (!testRollPitchYawRotations())
    return false;
  return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool
PlatformPhysicsTest::testRollOnlyRotations()
{
  HelicopterPlatform hp;
  double angle;
  double step = 0.09;
  double newRoll, pitch, yaw;
  for (size_t i = 1;; i++) {
    angle = i * step;
    if (angle >= PI_2)
      break;
    hp.rotate(angle, 0.0, 0.0);
    hp.getRollPitchYaw(newRoll, pitch, yaw);
    if (newRoll < 0.0)
      newRoll += PI_2;
    double diff = newRoll - angle;
    if (diff < -eps || diff > eps)
      return false;
  }
  return true;
}

bool
PlatformPhysicsTest::testPitchOnlyRotations()
{
  HelicopterPlatform hp;
  double angle;
  double step = 0.03;
  double roll, newPitch, yaw;
  for (size_t i = 1;; i++) {
    angle = i * step - PI_HALF;
    if (angle >= PI_HALF)
      break;
    hp.rotate(0.0, angle, 0.0);
    hp.getRollPitchYaw(roll, newPitch, yaw);
    double diff = newPitch - angle;
    if (diff < -eps || diff > eps)
      return false;
  }
  return true;
}

bool
PlatformPhysicsTest::testYawOnlyRotations()
{
  HelicopterPlatform hp;
  double angle;
  double step = 0.09;
  double roll, pitch, newYaw;
  for (size_t i = 1;; i++) {
    angle = i * step;
    if (angle >= PI_2)
      break;
    hp.rotate(0.0, 0.0, angle);
    hp.getRollPitchYaw(roll, pitch, newYaw);
    if (newYaw < 0.0)
      newYaw += PI_2;
    double diff = newYaw - angle;
    if (diff < -eps || diff > eps)
      return false;
  }
  return true;
}

bool
PlatformPhysicsTest::testRollPitchRotations()
{
  HelicopterPlatform hp;
  double angle1, angle2;
  double step1 = 0.09, step2 = 0.03;
  double newRoll, newPitch, yaw;
  for (size_t i = 1;; i++) {
    angle1 = i * step1;
    angle2 = i * step2 - PI_HALF;
    if (angle1 >= PI_2)
      break;
    hp.rotate(angle1, angle2, 0.0);
    hp.getRollPitchYaw(newRoll, newPitch, yaw);
    if (newRoll < 0.0)
      newRoll += PI_2;
    double diff1 = newRoll - angle1;
    double diff2 = newPitch - angle2;
    if (diff1 < -eps || diff1 > eps)
      return false;
    if (diff2 < -eps || diff2 > eps)
      return false;
  }
  return true;
}

bool
PlatformPhysicsTest::testRollYawRotations()
{
  HelicopterPlatform hp;
  double angle1, angle2;
  double step1 = 0.09, step2 = 0.09;
  double newRoll, pitch, newYaw;
  for (size_t i = 1;; i++) {
    angle1 = i * step1;
    angle2 = i * step2;
    if (angle1 >= PI_2)
      break;
    hp.rotate(angle1, 0.0, angle2);
    hp.getRollPitchYaw(newRoll, pitch, newYaw);
    if (newRoll < 0.0)
      newRoll += PI_2;
    if (newYaw < 0.0)
      newYaw += PI_2;
    double diff1 = newRoll - angle1;
    double diff2 = newYaw - angle2;
    if (diff1 < -eps || diff1 > eps)
      return false;
    if (diff2 < -eps || diff2 > eps)
      return false;
  }
  return true;
}

bool
PlatformPhysicsTest::testPitchYawRotations()
{
  HelicopterPlatform hp;
  double angle1, angle2;
  double step1 = 0.03, step2 = 0.09;
  double roll, newPitch, newYaw;
  for (size_t i = 1;; i++) {
    angle1 = i * step1 - PI_HALF;
    angle2 = i * step2;
    if (angle2 >= PI_2)
      break;
    hp.rotate(0.0, angle1, angle2);
    hp.getRollPitchYaw(roll, newPitch, newYaw);
    if (newYaw < 0.0)
      newYaw += PI_2;
    double diff1 = newPitch - angle1;
    double diff2 = newYaw - angle2;
    if (diff1 < -eps || diff1 > eps)
      return false;
    if (diff2 < -eps || diff2 > eps)
      return false;
  }
  return true;
}

bool
PlatformPhysicsTest::testRollPitchYawRotations()
{
  HelicopterPlatform hp;
  double angle1, angle2, angle3;
  double step1 = 0.09, step2 = 0.03, step3 = -0.06;
  double newRoll, newPitch, newYaw;
  for (size_t i = 1;; i++) {
    angle1 = i * step1;
    angle2 = i * step2 - PI_HALF;
    angle3 = i * step3;
    if (angle1 >= PI_2)
      break;
    hp.rotate(angle1, angle2, angle3);
    hp.getRollPitchYaw(newRoll, newPitch, newYaw);
    if (newRoll < 0.0)
      newRoll += PI_2;
    if (newYaw < 0.0)
      newYaw += PI_2;
    double diff1 = newRoll - angle1;
    double diff2 = newPitch - angle2;
    double diff3 = newYaw - (angle3 + PI_2);
    if (diff1 < -eps || diff1 > eps)
      return false;
    if (diff2 < -eps || diff2 > eps)
      return false;
    if (diff3 < -eps || diff3 > eps)
      return false;
  }
  return true;
}

}
