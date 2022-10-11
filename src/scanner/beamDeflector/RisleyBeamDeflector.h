#pragma once

#include "AbstractBeamDeflector.h"

#include "MathConverter.h"

/**
 * @brief Class representing a risley prisms beam deflector
 */
class RisleyBeamDeflector : public AbstractBeamDeflector {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
      
    double deltaT = 0;
    double pulseFreq = 0; 
    double time = 0;

    /** 
     * @brief Scan Angle (defined as the half angle)
     */
    double scanAngle = 0;
    double rotorSpeed_rad_1 = 0;
    double rotorSpeed_rad_2 = 0;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for risley prisms beam deflector
     * @see RisleyBeamDeflector::cfg_device_scanProduct
     * @see AbstractBeamDeflector::AbstractBeamDeflector(
     *  double, double, double)
     */
	RisleyBeamDeflector(
	    double scanAngleMax_rad,
        double rotorFreq_Hz_1,
        double rotorFreq_Hz_2
    ) :
	    AbstractBeamDeflector(
	        scanAngleMax_rad,
	        0,
	        0
        )
    {
        this->scanAngle = scanAngleMax_rad;
        this->rotorSpeed_rad_1 = rotorFreq_Hz_1 * 0.5 / M_PI;
        this->rotorSpeed_rad_2 = rotorFreq_Hz_2 * 0.5 / M_PI;
	}
    std::shared_ptr<AbstractBeamDeflector> clone() override;
    void _clone(std::shared_ptr<AbstractBeamDeflector> abd) override;

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

	// ***  GETTERS and SETTERS  *** //
	// ***************************** //
	/**
	 * @see AbstractBeamDeflector::setScanAngle_rad
	 */
	void setScanAngle_rad(double scanAngle_rad) override;
	/**
	 * @see AbstractBeamDeflector::setScanFreq_Hz
	 */
	void setScanFreq_Hz(double scanFreq_Hz) override;

};