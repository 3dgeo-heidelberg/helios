#pragma once

#include "Platform.h"

/**
 * @brief Class representing a moving platform
 */
class MovingPlatform : public Platform {

private:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Moving platform velocity vector
     */
	glm::dvec3 velocity = glm::dvec3(0, 0, 0);

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Moving platform default constructor
     */
    MovingPlatform() = default;
	std::shared_ptr<Platform> clone() override;
	void _clone(std::shared_ptr<Platform> p) override;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see Platform::applySettings
     */
	void applySettings(
	    std::shared_ptr<PlatformSettings> settings,
	    bool manual
    ) override;
	/**
	 * @see Platform::doSimStep
	 */
	void doSimStep(int simFrequency_hz) override;
	/**
	 * @see Platform::initLegManual
	 */
	void initLegManual() override;
	/**
	 * @brief Method to assist manual leg initialization for moving platform
	 * when it fails
	 * @see MovingPlatform::initLegManual
	 */
	void initLegManualIterative(); // To be used when initLegManual fails
	/**
	 * @see Platform::waypointReached
	 */
	bool waypointReached() override;
//        /**
//         * @see Platform::prepareSimulation
//         */
//        void prepareSimulation(int simFrequency_hz) override;
	/**
	 * @see Platform::getVelocity
	 */
	glm::dvec3 getVelocity() override {
		return velocity;
	}
	/**
	 * @brief Set velocity vector for moving platform
	 * @param v New velocity vector
	 */
	void setVelocity(glm::dvec3 v) {
		this->velocity = v;
	}
	/**
	 * @see Platform::canMove
	 */
    bool canMove() const override {return true;}
};