#pragma once

#include "MovingPlatform.h"

/**
 * @brief Class representing a simple phyiscs platform
 */
class SimplePhysicsPlatform : public MovingPlatform {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Engine force vector
     */
    glm::dvec3 mEngineForce = glm::dvec3(0, 0, 0);
    /**
     * @brief Gravity acceleration vector
     */
    glm::dvec3 mCfg_g_accel = glm::dvec3(0, 0, -9.81);
    /**
     * @brief How many meter does the platform move in each simulation step
    */
    double movePerSec_m_stepMagnitude = 0.0;
    /**
     * @brief Flag to store if the engine max thrust was reached in a given leg.
    The value is reset en each leg.
    */
    bool engineLimitReached = false;
    /**
     * @brief Flag to store if the user-provided movePerSec_m speed was achieved
     for a given leg.
    */
    bool userSpeedLimitReached = false;
public:
    /**
     * @brief Drag magnitude
     */
	double mCfg_drag = 1;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simple physics platform default constructor
     */
    SimplePhysicsPlatform() = default;
    std::shared_ptr<Platform> clone() override;
    void _clone(std::shared_ptr<Platform> p) override;

	// ***  M E T H O D S  *** //
	// *********************** //
  /**
   * @see Platform::prepareSimulation
   */
  void prepareSimulation(int simFrequency_hz) override;
	/**
	 * @brief Phyisics step for simple phyisics platform simulation
	 * @param simFrequency_hz Simulation frequency
	 */
	void doPhysicsStep(int simFrequency_hz);
	/**
	 * @see Platform::doSimStep
	 */
	void doSimStep(int simFrequency_hz) override;
	/**
	 * @brief Control step for simple phyisics platform simulation
	 * @param simFrequency_hz Simulation frequency
	 */
	virtual void doControlStep(int simFrequency_hz);

  void checkSpeedLimit();

};