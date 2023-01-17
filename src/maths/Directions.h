#pragma once

#include <glm/glm.hpp>

/**
* @brief Coordinate system convention
* @author Alberto M. Esmoris Pena
*/

class Directions {
public:
    // ***  CONSTANTS  *** //
    // ******************* //
    // Canonical basis
    /**
     * @brief <b>x</b> : left-right direction
     */
    static const glm::dvec3 right;
    /**
     * @brief <b>y</b> : forward-backward direction
     */
	static const glm::dvec3 forward;
	/**
     * @brief <b>z</b> : up-down direction
	 */
	static const glm::dvec3 up;

    // ARINC 705 norm
    /**
     * @brief Yaw rotation around \f$(0, 0, -1)\f$ as defined by the ARINC 705
     *  norm
     */
    static const glm::dvec3 yaw;
    /**
     * @brief Roll rotation around \f$(0, 1, 0)\f$ as defined by the ARINC 705
     *  norm
     */
    static const glm::dvec3 roll;
    /**
     * @brief Pitch rotation around \f$(1, 0, 0)\f$ as defined by the ARINC 705
     *  norm
     */
    static const glm::dvec3 pitch;
};