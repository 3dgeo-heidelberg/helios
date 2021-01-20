#pragma once

#include "AbstractBeamDeflector.h"

/**
 * @brief Class representing an oscillating mirror beam deflector
 */
class OscillatingMirrorBeamDeflector : public AbstractBeamDeflector {

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Scan product to limit maximum scan angle and the scanning
     * frequency
     */
    int cfg_device_scanProduct = 1000000;
    /**
     * @brief Current scan line pulse
     */
    int currentScanLinePulse = 0;

    // ***  CACHE ATTRIBUTES  *** //
    // ************************** //
    /**
     * @brief Pulses per scan line
     */
    int cached_pulsesPerScanline = 0;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for oscillating mirror beam deflector
     * @see OscillatingMirrorBeamDeflector::cfg_device_scanProduct
     * @see AbstractBeamDeflector::AbstractBeamDeflector(
     *  double, double, double)
     */
	OscillatingMirrorBeamDeflector(
	    double scanAngleMax_rad,
	    double scanFreqMax_Hz,
	    double scanFreqMin_Hz,
	    int scanProduct) :
	    AbstractBeamDeflector(
	        scanAngleMax_rad,
	        scanFreqMax_Hz,
	        scanFreqMin_Hz
        )
    {
		this->cfg_device_scanProduct = scanProduct;
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