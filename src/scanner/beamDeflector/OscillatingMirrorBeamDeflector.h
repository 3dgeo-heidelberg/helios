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
     * @see OscillatingMirrorBeamDeflector::computeTurningVelocityOffset
     * @see OscillatingMirrorBeamDeflector::cfg_device_turningPulsesf
     */
    int cfg_device_turningPulses = 100;
    /**
     * @brief Decimal encoding of cfg_device_turningPulses
     * @see OscillatingMirrorBeamDeflector::cfg_device_turningPulses
     */
    double cfg_device_turningPulsesf = 100.0f;

    // ***  CACHE ATTRIBUTES  *** //
    // ************************** //
    /**
     * @brief Pulses per scan line
     */
    int cached_pulsesPerScanline = 0;
    /**
     * @brief At which pulse step the turning around must be computed.
     * cached_threshold_pulse =
     *  cached_pulsesPerScanline - cfg_device_turningPulses
     * @see OscillatingMirrorBeamDeflector::computeTurningVelocityOffset
     * @see OscillatingMirrorBeamDeflector::cached_pulsesPerScanline
     * @see OscillatingMirrorBeamDeflector::cfg_device_turningPulses
     */
    int cached_thresholdPulse = -1;

    /**
     * @brief The starting point for first turning around stage on top peak
     * @see OscillatingMirrorBeamDeflector::cached_thresholdPulse
     * @see OscillatingMirrorBeamDeflector::computeTurningVelocityOffset
     */
    int cached_aHalfThresholdPulse = -1;
    /**
     * @brief The starting point for second turning around stage on top peak
     * @see OscillatingMirrorBeamDeflector::cached_thresholdPulse
     * @see OscillatingMirrorBeamDeflector::computeTurningVelocityOffset
     */
    int cached_halfThresholdPulse = -1;
    /**
     * @brief The ending point for second turning around stage on top peak
     * @see OscillatingMirrorBeamDeflector::cached_thresholdPulse
     * @see OscillatingMirrorBeamDeflector::computeTurningVelocityOffset
     */
    int cached_bHalfThresholdPulse = -1;

    /**
     * @brief Half of the turning pulses, because they are required to compute
     *  turning velocity offset
     * @see OscillatingMirrorBeamDeflector::computeTurningVelocityOffset
     */
    double cached_halfTurningPulses = -1.0;

    /**
     * @brief Offset scale factor used to speed-up turning velocity offset
     *  computations.
     * cached_offsetScaleFactor =
     *  cached_halfTurningPulses * cached_angleBetweenPulses_rad
     * @see OscillatingMirrorBeamDeflector::cached_halfTurningPulses
     * @see OscillatingMirrorBeamDeflector::cached_angleBetweenPulses_rad
     */
    double cached_offsetScaleFactor = -1.0;

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

	/**
	 * @brief Compute the turning velocity offset.
	 *
	 * The turning velocity offset can be expressed as a set of 4 equations.
	 * For this purpose, let \f$l\f$ be the number of pulses per scan line,
	 *  \f$c\f$ the current scan line pulse, \f$n\f$ the number of pulses
	 *  the turning around takes, \f$\Delta\f$ the angle between pulses in
	 *  radians and \f$\epsilon_{n}(x)\f$ the function corresponding to
	 *  turning velocity offset.
	 *
	 * Equation for down to bottom peak offset:
	 * \f[
	 * \left\{\begin{array}{lll}
	 *  x & = & l - c \\
	 *  \epsilon_{n}(x) & = &
	 *      \frac{n\Delta}{2}
	 *      \left(1 - \frac{x}{n}\right)^2
	 * \end{array}\right.
	 * \f]
	 *
	 * Equation for up from bottom peak offset:
	 * \f[
	 * \left\{\begin{array}{lll}
	 *  x & = & c \\
	 *  \epsilon_{n}(x) & = &
	 *      \frac{n\Delta}{2}
	 *      \left(1 - \frac{x}{n}\right)^2
	 * \end{array}\right.
	 * \f]
	 *
	 * Equation for up to top peak offset:
	 * \f[
	 * \left\{\begin{array}{lll}
	 *  x & = & c - \frac{l}{2} + n \\
	 *  \epsilon_{n}(x) & = &
	 *      - \frac{n\Delta}{2}
	 *      \left(\frac{x}{n}\right)^2
	 * \end{array}\right.
	 * \f]
	 *
	 * Equation for down from top peak offset:
	 * \f[
	 * \left\{\begin{array}{lll}
	 *  x & = & \frac{l}{2} + n - c \\
	 *  \epsilon_{n}(x) & = &
	 *      - \frac{n\Delta}{2}
	 *      \left(\frac{x}{n}\right)^2
	 * \end{array}\right.
	 * \f]
	 *
	 * The function behavior of oscillating mirror velocity has been taken
	 * from:
     *  "The geometry of Airborne Laser Scanning in a Kinematical Framework"
     *      by Andreas Roncat. See pages 54 to 56 for more information.
	 */
	double computeTurningVelocityOffset();

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