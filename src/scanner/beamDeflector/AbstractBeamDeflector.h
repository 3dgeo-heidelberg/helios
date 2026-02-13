#pragma once

#include <memory>
#include <string>

#include <glm/glm.hpp>

#include "ScannerSettings.h"

#include "maths/Rotation.h"

/**
 * @brief Base abstract class for beam deflectors
 */
class AbstractBeamDeflector
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  // Device definition variables:
  /**
   * @brief Minimum scanning frequency (hertz)
   */
  double cfg_device_scanFreqMax_Hz = 0;
  /**
   * @brief Maximum scanning frequency (hertz)
   */
  double cfg_device_scanFreqMin_Hz = 0;
  /**
   * @brief Maximum scanning angle (radians)
   */
  double cfg_device_scanAngleMax_rad = 0;

  // Setting variables:
  /**
   * @brief Scanning frequency (hertz)
   */
  double cfg_setting_scanFreq_Hz = 0;
  /**
   * @brief Scanning angle (radians)
   */
  double cfg_setting_scanAngle_rad = 0;
  /**
   * @brief Minimum vertical scanning angle (radians)
   */
  double cfg_setting_verticalAngleMin_rad = 0;
  /**
   * @brief Maximum vertical scanning angle (radians)
   */
  double cfg_setting_verticalAngleMax_rad = 0;
  /**
   * @brief Optional warmup phase (seconds) consumed by compatible deflectors
   */
  double cfg_setting_warmupPhase_s = 0;

  // Stat variables:
  /**
   * @brief Current beam angle (radians)
   */
  double state_currentBeamAngle_rad = 0;
  /**
   * @brief Angle differential (radians)
   */
  double state_angleDiff_rad = 0;

  // ############# BEGIN "state cache" members ##############
  // Members with "_cached" prefix contain values that are derived from state
  // variables and required multiple times per sim step at different places in
  // the code. In order to avoid unneccessary re-computations of the same value,
  // they are cached in special variables:
  /**
   * @brief Angle between pulses (radians)
   */
  double cached_angleBetweenPulses_rad;

  /**
   * @brief Relative emitter attitude
   */
  Rotation cached_emitterRelativeAttitude = Rotation(glm::dvec3(1, 0, 0), 0);

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Base constructor for beam deflectors
   * @see AbstractBeamDeflector::scanAngleMax_rad
   * @see AbstractBeamDeflector::scanFreqMax_Hz
   * @see AbstractBeamDeflector::scanFreqMin_Hz
   */
  AbstractBeamDeflector(double scanAngleMax_rad,
                        double scanFreqMax_Hz,
                        double scanFreqMin_Hz)
  {
    cfg_device_scanAngleMax_rad = scanAngleMax_rad;
    cfg_device_scanFreqMax_Hz = scanFreqMax_Hz;
    cfg_device_scanFreqMin_Hz = scanFreqMin_Hz;
    state_currentBeamAngle_rad = 0;
  }

  virtual ~AbstractBeamDeflector() {}
  virtual std::shared_ptr<AbstractBeamDeflector> clone() = 0;
  virtual void _clone(std::shared_ptr<AbstractBeamDeflector> abd);

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Apply given scanner settings to beam deflector
   * @param settings Scanner settings
   * @see ScannerSettings
   */
  virtual void applySettings(std::shared_ptr<ScannerSettings> settings);
  /**
   * @brief Check if last pulse left device (true) or not (false)
   * @return True if last pulse left device, false otherwise
   */
  virtual bool lastPulseLeftDevice();
  /**
   * @brief Restart the deflector.
   * This method is meant to be invoked whenever the deflector needs to be
   *  restarted. Which generally occurs when the previous leg was inactive.
   */
  virtual void restartDeflector();
  /**
   * @brief Perform computations for current simulation step
   */
  virtual void doSimStep() = 0;

  /**
   * @brief Method to get optics type of deflector
   */
  virtual std::string getOpticsType() const = 0;

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Get the relative emitter attitude
   * @return Relative emitter attitude
   */
  Rotation getEmitterRelativeAttitude();
  /**
   * @brief  Obtain the exact relative emitter attitude. By default, it is
   *  the relative emitter attitude. However, beam deflectors simulating
   *  mechanical errors need to track the exact emitter relative attitude
   *  separately because the typical one includes simulated mechanical
   *  errors.
   * @return The exact emitter relative attitude
   * @see AbstractBeamDeflector::getEmitterRelativeAttitude
   */
  virtual Rotation getExactEmitterRelativeAttitude()
  {
    return getEmitterRelativeAttitude();
  }
  /**
   * @brief Get the relative emitter attitude by referencce
   * @return Reference to relative emitter attitude
   */
  Rotation& getEmitterRelativeAttitudeByReference()
  {
    return this->cached_emitterRelativeAttitude;
  };
  /**
   * @brief Set the scan angle
   * @param scanAngle_rad New scan angle (radians)
   */
  virtual void setScanAngle_rad(double scanAngle_rad);
  /**
   * @brief Set the scanning frequency
   * @param scanFreq_hz New scanning frequency (hertz)
   */
  virtual void setScanFreq_Hz(double scanFreq_hz);
  /**
   * @brief Set warmup phase for deflector internals
   * @param warmupPhase_s Warmup phase in seconds
   */
  virtual void setWarmupPhase_s(double warmupPhase_s);
  /**
   * @brief Get warmup phase currently set on deflector
   * @return Warmup phase in seconds
   */
  virtual double getWarmupPhase_s() { return cfg_setting_warmupPhase_s; }
  /**
   * @brief Get the exact current beam angle.
   *
   * By default this function returns the base current beam angle
   *  AbstractBeamDeflector::state_currentBeamAngle_rad but deflectors
   *  modeling measurement error will need to override this to provide the
   *  beam angle without error
   *
   * @return Current beam angle (radians) with no error
   * @see AbstractBeamDeflector::state_currentBeamAngle_rad
   */
  virtual double getCurrentExactBeamAngle()
  {
    return state_currentBeamAngle_rad;
  }

  /**
   * @brief Check whether the deflector simulates mechanical errors
   *  (true) or not (false).
   * @return True if the deflectors simulates mechanical errors,
   *  false otherwise
   */
  virtual bool hasMechanicalError() { return false; }
};
