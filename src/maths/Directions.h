#pragma once

#include <glm/glm.hpp>

/**
* @brief Coordinate system convention
*
*/

class Directions {
public:
    // ***  CONSTANTS  *** //
    // ******************* //
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
};