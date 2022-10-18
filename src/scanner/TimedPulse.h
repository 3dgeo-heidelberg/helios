#pragma once

#include <Pulse.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a time aware laser pulse
 */
class TimedPulse : public Pulse{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The start time of the pulse (in nanoseconds).
     *  Typically in real world applications it is the GPS time.
     */
    double time_ns;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Time laser pulse constructor
     * @see Pulse
     * @see Pulse::origin
     * @see Pulse::attitude
     * @see TimedPulse::time_ns
     */
    TimedPulse(
        glm::dvec3 const &origin,
        Rotation const &attitude,
        double const time_ns
    ) :
        Pulse(origin, attitude),
        time_ns(time_ns)
    {}
    virtual ~TimedPulse() = default;


    // *** GETTERs and SETTERs  *** //
    // **************************** //
    /**
     * @brief Obtain the start time of the pulse (in nanoseconds)
     * @return The start time of the pulse (in nanoseconds)
     * @see TimedPulse::time_ns
     */
    inline double getTime() const {return time_ns;}
    /**
     * @brief Set the start time of the pulse
     * @param time_ns The new start time for the pulse (in nanoseconds)
     * @see TimedPulse::time_ns
     */
    inline void setTime(double const time_ns) {this->time_ns = time_ns;}
};