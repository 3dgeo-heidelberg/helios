#pragma once

#include <glm/glm.hpp>

#include "Asset.h"

/**
 * @brief Class representing platform settings
 */
class PlatformSettings : public Asset {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Position x coordinate
     */
	double x = 0;
	/**
	 * @brief Position y coordinate
	 */
	double y = 0;
	/**
	 * @brief Position z coordinate
	 */
	double z = 0;
    /**
     * @brief Flag
     */
    bool yawAtDepartureSpecified = false;
	/**
	 * @brief Yaw angle (in radians) at platform departure
	 */
	double yawAtDeparture = 0.0;

	/**
	 * @brief On ground flag
	 * @see Platform::onGround
	 */
	bool onGround = false;
	/**
	 * @brief Stop and turn flag
	 * @see Platform::stopAndTurn
	 */
	bool stopAndTurn = true;
	/**
	 * @brief Smooth turn
	 * @see Platform::smoothTurn
	 */
	bool smoothTurn = false;
	/**
	 * @brief Slowdown enabled flag
	 * @see Platform::slowdownEnabled
	 */
	bool slowdownEnabled = true;

	// 100 meter per sec are 360 km/h:
	/**
	 * @brief Movement per seconds (in meters)
	 */
	double movePerSec_m = 70;

	// ***  GETTERS and SETTERS  *** //
	// ***************************** //
	/**
	 * @brief Obtain position as 3D vector
	 * @return Position as 3D vector
	 */
	glm::dvec3 getPosition() {
		return glm::dvec3(x, y, z);
	}

	/**
	 * @brief Set position from 3D vector
	 * @param dest Position as 3D vector
	 */
	void setPosition(glm::dvec3 dest) {
		x = dest.x;
		y = dest.y;
		z = dest.z;
	}

};