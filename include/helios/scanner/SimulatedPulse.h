#pragma once

#include <helios/scanner/TimedPulse.h>

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Class representing a simulated laser pulse
 */
class SimulatedPulse : public TimedPulse
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The index of the simulation leg generating the pulse
   *
   * Which leg the FullWaveformPulseRunnable belongs to.
   *
   * While this attribute is not strictly necessary for the
   * FullWaveformPulseRunnable to do its job, it really helps with
   * tracing and debugging concurrency issues.
   *
   * For instance, to track what is going on with end of leg
   * FullWaveformPulseRunnable threads while a new leg is being started.
   *
   * This attribute could be safely removed without degenerating class
   * mechanics. So, if in the future it is not wanted any more, feel free
   * to remove it.
   */
  unsigned int legIndex;
  /**
   * @brief The number of the pulse, counting from the perspective of the
   *  concrete scanning device which generated the pulse
   */
  int pulseNumber;
  /**
   * @brief The index of the scanning device inside the scanner context
   *  generating the pulse
   * @see Scanner
   * @see SingleScanner
   * @see MultiScanner
   * @see ScanningDevice
   */
  size_t deviceIndex;
  /**
   * @brief Flag that must be true when the simulated pulse comes from a
   *  simulation with mechanical errors, false otherwise.
   * @see ScanningDevice::hasMechanicalError
   */
  bool mechanicalError = false;

  /**
   * @brief The attitude without mechanical errors. When there is no
   *  mechanical error simulation, it will match the Pulse::attitude
   *  attribute
   * @see Pulse::attitude
   * @see SimulatedPulse::mechanicalError
   */
  Rotation exactAttitude;

  /**
   * @brief The mechanical range error. When there is no mechanical error
   *  simulation, it will be exactly 0.
   * @see SimulatedPulse::mechanicalError
   * @see ScanningDevice::evalRangeErrorExpression
   */
  double mechanicalRangeError = 0.0;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simulated laser pulse constructor
   * @see TimedPulse
   * @see Pulse::origin
   * @see Pulse::attitude
   * @see TimedPulse::time_ns
   * @see SimulatedPulse::legIndex
   * @see SimulatedPulse::pulseNumber
   * @see SimulatedPulse::deviceIndex
   */
  SimulatedPulse(glm::dvec3 const& origin,
                 Rotation const& attitude,
                 double const time_ns,
                 unsigned int legIndex,
                 int const pulseNumber,
                 size_t const deviceIndex)
    : TimedPulse(origin, attitude, time_ns)
    , legIndex(legIndex)
    , pulseNumber(pulseNumber)
    , deviceIndex(deviceIndex)
    , mechanicalError(false)
    , exactAttitude(attitude)
    , mechanicalRangeError(0.0)
  {
  }
  /**
   * @brief Simulated laser pulse constructor for simulations including
   *  mechanical errors
   * @see SimulatedPulse::SimulatedPulse
   * @see SimulatedPulse::mechanicalError
   * @see SimulatedPulse::exactAttitude
   */
  SimulatedPulse(glm::dvec3 const& origin,
                 Rotation const& attitude,
                 Rotation const& exactAttitude,
                 double const mechanicalRangeError,
                 double const time_ns,
                 unsigned int legIndex,
                 int const pulseNumber,
                 size_t const deviceIndex)
    : TimedPulse(origin, attitude, time_ns)
    , legIndex(legIndex)
    , pulseNumber(pulseNumber)
    , deviceIndex(deviceIndex)
    , mechanicalError(true)
    , exactAttitude(exactAttitude)
    , mechanicalRangeError(mechanicalRangeError)
  {
  }
  ~SimulatedPulse() override = default;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the leg index of the pulse
   * @return The leg index of the pulse
   * @see SimulatedPulse::legIndex
   */
  inline unsigned int getLegIndex() const { return legIndex; }
  /**
   * @brief Set the leg index of the pulse
   * @param legIndex The new leg index for the pulse
   * @see SimulatedPulse::legIndex
   */
  inline void setLegIndex(unsigned int const legIndex)
  {
    this->legIndex = legIndex;
  }
  /**
   * @brief Obtain the pulse number wrt emitter scanning device
   * @return The pulse number wrt emitter scanning device
   * @see SimulatedPulse::pulseNumber
   */
  inline int getPulseNumber() const { return pulseNumber; }
  /**
   * @brief Set the pulse number wrt emitter scanning device
   * @param pulseNumber The new pulse number wrt emitter scanning device
   * @see SimulatedPulse::pulseNumber
   */
  inline void setPulseNumber(int const pulseNumber)
  {
    this->pulseNumber = pulseNumber;
  }
  /**
   * @brief Obtain the device index of the emitter scanning device
   * @return The device index of the emitter scanning device
   * @see SimulatedPulse::deviceIndex
   */
  inline size_t getDeviceIndex() const { return deviceIndex; }
  /**
   * @brief Set the device index of the emitter scanning device
   * @param deviceIndex The new device index of the emitter scanning device
   * @see SimulatedPulse::deviceIndex
   */
  inline void setDeviceIndex(size_t const deviceIndex)
  {
    this->deviceIndex = deviceIndex;
  }
  /**
   * @brief Obtain the exact attitude (ignoring mechanical errors) of the
   *  ray
   * @return Copy of the exact attitude of the ray
   * @see SimulatedPulse::exactAttitude
   */
  inline Rotation getExactAttitude() const { return exactAttitude; }
  /**
   * @brief Obtain the exact attitude of the ray by reference
   * @return Reference to the exact attitude of the ray
   * @see SimulatedPulse::getExactAttitude
   */
  inline Rotation& getExactAttitudeRef() { return exactAttitude; }
  /**
   * @biref Obtain the simulated mechanical range error
   * @return Simulated mechanical range error
   * @see SimulatedPulse::mechanicalRangeError
   */
  inline double getMechanicalRangeError() const { return mechanicalRangeError; }
  /**
   * @brief Check whether the simulated pulse contains mechanical errors
   *  (True) or not (False).
   * @return True if the simulated pulse contains mechanical errors, False
   *  otherwise
   * @see SimulatedPulse::mechanicalError
   */
  inline bool hasMechanicalError() const { return mechanicalError; }

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Compute the director vector of the ray/beam without mechanical
   *  errors.
   * @return The director vector of the ray/beam without mechanical errors.
   */
  inline glm::dvec3 computeExactDirection()
  {
    return exactAttitude.applyTo(Directions::forward);
  }
};
