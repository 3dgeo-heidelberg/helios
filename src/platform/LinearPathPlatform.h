#pragma once

#include "MovingPlatform.h"

/**
 * @brief Class representing a linear path platform
 */
class LinearPathPlatform : public MovingPlatform {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for linear path platform
     */
    LinearPathPlatform() = default;
	std::shared_ptr<Platform> clone() override;
	void _clone(std::shared_ptr<Platform> p) override;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see Platform::doSimStep
     */
	void doSimStep(int simFrequency_hz) override;
	/**
	 * @see Platform::setDestination
	 */
	void setDestination(glm::dvec3 dest) override;
};