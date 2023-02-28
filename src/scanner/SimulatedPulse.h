#pragma once

#include <TimedPulse.h>

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Class representing a simulated laser pulse
 */
class SimulatedPulse : public TimedPulse{
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
    SimulatedPulse(
        glm::dvec3 const &origin,
        Rotation const &attitude,
        double const time_ns,
        unsigned int legIndex,
        int const pulseNumber,
        size_t const deviceIndex
    ) :
        TimedPulse(origin, attitude, time_ns),
        legIndex(legIndex),
        pulseNumber(pulseNumber),
        deviceIndex(deviceIndex)
    {}
    virtual ~SimulatedPulse() = default;


    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the leg index of the pulse
     * @return The leg index of the pulse
     * @see SimulatedPulse::legIndex
     */
    inline unsigned int getLegIndex() const {return legIndex;}
    /**
     * @brief Set the leg index of the pulse
     * @param legIndex The new leg index for the pulse
     * @see SimulatedPulse::legIndex
     */
    inline void setLegIndex(unsigned int const legIndex)
    {this->legIndex = legIndex;}
    /**
     * @brief Obtain the pulse number wrt emitter scanning device
     * @return The pulse number wrt emitter scanning device
     * @see SimulatedPulse::pulseNumber
     */
    inline int getPulseNumber() const {return pulseNumber;}
    /**
     * @brief Set the pulse number wrt emitter scanning device
     * @param pulseNumber The new pulse number wrt emitter scanning device
     * @see SimulatedPulse::pulseNumber
     */
    inline void setPulseNumber(int const pulseNumber)
    {this->pulseNumber = pulseNumber;}
    /**
     * @brief Obtain the device index of the emitter scanning device
     * @return The device index of the emitter scanning device
     * @see SimulatedPulse::deviceIndex
     */
    inline size_t getDeviceIndex() const {return deviceIndex;}
    /**
     * @brief Set the device index of the emitter scanning device
     * @param deviceIndex The new device index of the emitter scanning device
     * @see SimulatedPulse::deviceIndex
     */
    inline void setDeviceIndex(size_t const deviceIndex)
    {this->deviceIndex = deviceIndex;}
};