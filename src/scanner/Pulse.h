#pragma once

#include <Rotation.h>
#include <maths/Directions.h>

#include <glm/glm.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a laser pulse
 */
class Pulse {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The origin of the pulse, typically named as the point \f$o\f$
     */
    glm::dvec3 origin;
    /**
     * @brief The attitude of the ray
     */
    Rotation attitude;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Laser pulse constructor
     * @see Pulse::origin
     * @see Pulse:attitude
     */
    Pulse(glm::dvec3 const &origin, Rotation const &attitude) :
        origin(origin),
        attitude(attitude)
    {}
    virtual ~Pulse() = default;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the origin of the pulse
     * @return Copy of the origin of the pulse
     * @see Pulse::origin
     */
    inline glm::dvec3 getOrigin() const {return origin;}
    /**
     * @brief Obtain the origin of the pulse by reference
     * @return Reference to the origin of the pulse
     * @see Pulse::origin
     */
    inline glm::dvec3 & getOriginRef() {return origin;}
    /**
     * @brief Set the origin of the pulse
     * @param origin The new origin for the pulse
     * @see Pulse::origin
     */
    inline void setOrigin(glm::dvec3 const & origin) {this->origin = origin;}
    /**
     * @brief Obtain the attitude of the ray
     * @return Copy of the attitude of the ray
     * @see Pulse::attitude
     */
    inline Rotation getAttitude() const {return attitude;}
    /**
     * @brief Obtain the attitude of the ray by reference
     * @return Reference to the attitude of the ray
     */
    inline Rotation & getAttitudeRef() {return attitude;}
    /**
     * @brief Set the attitude of the ray
     * @param attitude The new attitude for the ray
     */
    inline void setAttitude(Rotation const & attitude)
    {this->attitude = attitude;}


    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @brief Compute the director vector of the ray/beam
     * @return The director vector of the ray/beam
     */
    inline glm::dvec3 computeDirection()
    {return attitude.applyTo(Directions::forward);}

};