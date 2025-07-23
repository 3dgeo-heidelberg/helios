#pragma once

#include "ScannerSettings.h"

#include "maths/Rotation.h"

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * Class representing a scanner head
 */
class ScannerHead
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Maximum rotation per second (radians)
   */
  double cfg_device_rotatePerSecMax_rad = 0;

  // Settings:
  /**
   * @brief Rotation per second (radians)
   */
  double cfg_setting_rotatePerSec_rad = 0;
  /**
   * @brief Rotation stop angle (radians)
   */
  double cfg_setting_rotateStop_rad = 0;
  /**
   * @brief Rotation start angle (radians)
   */
  double cfg_setting_rotateStart_rad = 0;
  /**
   * @brief Difference between rotation stop and start angles (radians)
   */
  double cfg_setting_rotateRange_rad = 0;

  // State variables:
  /**
   * @brief Current rotation angle (radians)
   */
  double state_currentRotateAngle_rad = 0;

  // Cache variables:
  /**
   * @brief Relative scanner head mount attitude
   */
  Rotation cached_mountRelativeAttitude = Rotation(glm::dvec3(0, 1, 0), 0);

public:
  // Device definition:
  /**
   * @brief Rotation axis
   */
  glm::dvec3 cfg_device_rotateAxis = glm::dvec3(1, 0, 0);

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Scanner head constructor
   * @see ScannerHead::cfg_device_rotateAxis
   * @see ScannerHead::cfg_device_rotatePerSecMax_rad
   */
  ScannerHead(glm::dvec3 headRotationAxis, double headRotatePerSecMax_rad)
  {
    this->cfg_device_rotateAxis = headRotationAxis;
    this->cfg_device_rotatePerSecMax_rad = headRotatePerSecMax_rad;
  }
  virtual ~ScannerHead() {}

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Apply scanner settings to the scanner head
   * @param settings Scanner settings to be applied
   */
  virtual void applySettings(std::shared_ptr<ScannerSettings> settings);
  /**
   * @brief Perform computations for current simulation step
   * @param pulseFreq_Hz Pulse frequency (hertz)
   */
  virtual void doSimStep(double pulseFreq_Hz);

  /**
   * @brief Check if rotation has been completed.
   *
   * Rotation is considered to be completed when the rotation stop angle
   * has been reached
   *
   * @return True of rotation has been completed, false otherwise
   * @see ScannerHead::cfg_setting_rotateStop_rad
   */
  virtual bool rotateCompleted();

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain the relative mount attitude
   * @return Relative mount attitude
   * @see ScannerHead::cached_mountRelativeAttitude
   */
  Rotation getMountRelativeAttitude();
  /**
   * @brief Obtain the exact relative mount attitude. By default, it is the
   *  relative mount attitude. However, scanner heads simulating mechanical
   *  errors need to track the exact relative mount attitude separately
   *  because the typical one includes simulated mechanical errors.
   * @return The exact relative mount attitude
   * @see ScannerHead::getMountRelativeAttitude
   */
  virtual Rotation getExactMountRelativeAttitude()
  {
    return getMountRelativeAttitude();
  }
  /**
   * @see Obtain the relative mount attitude by reference
   * @return Reference to the relative mount attitude
   * @see ScannerHead::cached_mountRelativeAttitude
   */
  Rotation& getMountRelativeAttitudeByReference()
  {
    return this->cached_mountRelativeAttitude;
  }

  /**
   * @brief Get the maximum rotation per second
   * @return Maximum rotation per second (radians)
   * @see ScannerHead::cfg_device_rotatePerSecMax_rad
   */
  double getRotatePerSecMax() { return cfg_device_rotatePerSecMax_rad; }
  /**
   * @brief Set the maximum rotation per second
   * @param rotatePerSecMax New maximum rotation per second (radians)
   * @see ScannerHead::cfg_device_rotatePerSecMax_rad
   */
  void setRotatePerSecMax(double rotatePerSecMax)
  {
    this->cfg_device_rotatePerSecMax_rad = rotatePerSecMax;
  }

  /**
   * @brief Get the current rotation angle
   * @return Current rotation angle (radians)
   * @see ScannerHead::state_currentRotateAngle_rad
   */
  double getRotateCurrent() { return state_currentRotateAngle_rad; }
  /**
   * @brief Get the exact current rotation angle.
   *
   * By default this function returns the same than
   *  ScannerHead::getRotateCurrent but scanner heads modeling measurement
   *  error will need to override this to provide the rotation angle without
   *  error.
   *
   * @return Current rotation angle (radians) with no error
   * @see ScannerHead::getRotateCurrent
   */
  virtual double getExactRotateCurrent() { return getRotateCurrent(); }
  /**
   * @brief Set the current rotation angle
   * @param angle_rad New current rotation angle (radians)
   * @see ScannerHead::state_currentRotateAngle_rad
   */
  virtual void setCurrentRotateAngle_rad(double angle_rad);

  /**
   * @brief Get rotation per second
   * @return Rotation per second (radians)
   * @see ScannerHead::cfg_setting_rotatePerSec_rad
   */
  double getRotatePerSec_rad() { return cfg_setting_rotatePerSec_rad; }
  /**
   * @brief Set rotation per second
   * @param rotateSpeed_rad New rotation per second (radians)
   * @see ScannerHead::rotateSpeed_rad
   */
  void setRotatePerSec_rad(double rotateSpeed_rad);

  /**
   * @brief Get rotation stop angle
   * @return Rotation stop angle (radians)
   * @see ScannerHead::cfg_setting_rotateStop_rad
   */
  double getRotateStop() { return cfg_setting_rotateStop_rad; }
  /**
   * @brief Set rotation stop angle
   * @param rotateStop New rotation stop angle (radians)
   * @see ScannerHead::cfg_setting_rotateStop_rad
   */
  void setRotateStop(double rotateStop)
  {
    this->cfg_setting_rotateStop_rad = rotateStop;
  }

  /**
   * @brief Get rotation start angle
   * @return Rotation start angle (radians)
   * @see ScannerHead::cfg_setting_rotateStart_rad
   */
  double getRotateStart() { return cfg_setting_rotateStart_rad; }
  /**
   * @brief Set rotation start angle
   * @param rotateStart New rotation start angle (radians)
   * @see ScannerHead::cfg_setting_rotateStart_rad
   */
  void setRotateStart(double rotateStart)
  {
    this->cfg_setting_rotateStart_rad = rotateStart;
  }

  /**
   * @brief Get rotation range
   * @return Rotation range (radians)
   * @see ScannerHead::cfg_setting_rotateRange_rad
   */
  double getRotateRange() { return cfg_setting_rotateRange_rad; }
  /**
   * @brief Set rotation range
   * @param rotateRange New rotation range
   * @see ScannerHead::cfg_setting_rotateRange_rad
   */
  void setRotateRange(double rotateRange)
  {
    this->cfg_setting_rotateRange_rad = rotateRange;
  }
  /**
   * @brief Check whether the scanner head simulates mechanical errors
   *  (true) or not (false).
   * @return True if the scanner head simulates mechanical errors,
   *  false otherwise
   */
  virtual bool hasMechanicalError() { return false; }
};
