#pragma once

#include "AbstractBeamDeflector.h"

/**
 * @brief Class representing a fiber array beam deflector
 */
class FiberArrayBeamDeflector : public AbstractBeamDeflector {
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Number of fibers composing the deflector
     */
	int cfg_device_numFibers = 32;
	/**
	 * @brief Index of current fiber
	 */
	int state_currentFiber = 0;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for fiber array beam deflector
     * @param numFibers Number of fibers composing the deflector
     * @see AbstractBeamDeflector::AbstractBeamDeflector(
     *  double, double, double)
     */
	FiberArrayBeamDeflector(
	    double scanAngleMax_rad,
	    double scanFreqMax_Hz,
	    double scanFreqMin_Hz,
	    int numFibers) :
	    AbstractBeamDeflector(
	        scanAngleMax_rad,
	        scanFreqMax_Hz,
	        scanFreqMin_Hz
        )
    {
		cfg_device_numFibers = numFibers;
	}
    std::shared_ptr<AbstractBeamDeflector> clone();
    void _clone(std::shared_ptr<AbstractBeamDeflector> abd);

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see AbstractBeamDeflector::applySettings
     */
	void applySettings(std::shared_ptr<ScannerSettings> settings) override;
	/**
	 * @see AbstractBeamDeflector::doSimStep
	 */
	void doSimStep() override;

	// ***  GETTERS and SETTERS *** //
	// **************************** //
    /**
     * @brief Set the number of fibers
     * @param numFibers New number of fibers
     */
    void setNumFibers(int numFibers);
};