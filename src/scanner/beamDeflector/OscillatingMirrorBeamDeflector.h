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

    /**
     * @brief How many pulses are required for the turning around
     *
     * <b><span style="color:red">CAUTION</span></b> If number of turning
     *  pulses \f$>\f$ number of pulses per scan line, then the oscillating
     *  mirror beam deflector will not work properly. Because there is not
     *  enough time to complete both necessary turnings per scanline.
     */
    int cfg_device_turningPulses = 100;

    // ***  CACHE ATTRIBUTES  *** //
    // ************************** //
    /**
     * @brief Pulses per scan line
     */
    int cached_pulsesPerScanline = 0;
    /**
     * @brief The number of the pulse at the half of the scanline
     */
    int cached_halfScanlinePulse;
    /**
     * @brief The number of the scanline pulse where the first accelerate
     *  operation starts. The first accelerate operation occurs during the
     *  positive monotonic region of the bitonic scanline.
     */
    int cached_firstAccelerateScanlinePulse;
    /**
     * @brief The number of the scanline pulse where the first constant linear
     *  operation starts. The first linear operation occurs during the positive
     *  monotonic region of the bitonic scanline.
     */
    int cached_firstLinearScanlinePulse;
    /**
     * @brief The number of the scanline pulse where the first decelerate
     *  operation starts. The first decelerate operation occurs during the
     *  positive monotonic region of the bitonic scanline.
     */
    int cached_firstDecelerateScanlinePulse;
    /**
     * @brief The number of scanline pulse where the second accelerate
     *  operation starts. The second accelerate operation occurs duting the
     *  negative monotonic region of the bitonic scanline.
     */
    int cached_secondAccelerateScanlinePulse;
    /**
     * @brief The number of scanline pulse where the second constant linear
     *  operation starts. The second linear operation occurs during the
     *  negative monotonic region of the bitonic scanline.
     */
    int cached_secondLinearScanlinePulse;
    /**
     * @brief The number of the scanline pulse where the second decelerate
     *  operation starts. The second decelerate operation occurs during the
     *  negative monotonic region of the bitonic scanline.
     */
    int cached_secondDecelerateScanlinePulse;


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

    // ***  MAIN METHODS  *** //
    // ********************** //
    /**
     * @see AbstractBeamDeflector::applySettings
     */
	void applySettings(std::shared_ptr<ScannerSettings> settings) override;
	/**
	 * @see AbstractBeamDeflector::doSimStep
	 */
	void doSimStep() override;

	// ***  UTIL METHODS  *** //
	// ********************** //
	/**
	 * @brief Update the current beam angle at each simulation step
	 */
	void updateBeamAngle();
	/**
	 * @brief Compute the beam angle acceleration
	 * @param p Current pulse
	 * @param pa Start pulse of the acceleration stage (inclusive)
	 * @param pb End pulse of the acceleration stage (exclusive)
	 * @param sign Specify whether the beam angle is increasing (positive, 1)
	 *  or decreasing (negative, -1)
	 * @return
	 */
	void accelerateBeamAngle(
	    double const p,
	    double const pa,
	    double const pb,
        double const sign
    );
	/**
	 * @brief Compute the beam angle linear behavior
	 * @param p Current pulse
	 * @param pa Start pulse of the linear stage (inclusive)
	 * @param pb End pulse of the linear stage (exclusive)
	 * @param sign Specify whether the beam angle is increasing (positive, 1)
	 *  or decreasing (negative, -1)
	 */
	void linearBeamAngle(
	    double const p,
	    double const pa,
	    double const pb,
	    double const sign
    );
	/**
	 * @brief Compute the beam angle deceleration
	 * @param p Current pulse
	 * @param pa Start pulse of the deceleration stage (inclusive)
	 * @param pb End pulse of the deceleration stage (exclusive)
	 * @param sign Specify whether the beam angle is increasing (positive, 1)
	 *  or decreasing (negative, -1)
	 */
	void decelerateBeamAngle(
	    double const p,
	    double const pa,
	    double const pb,
	    double const sign
    );


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