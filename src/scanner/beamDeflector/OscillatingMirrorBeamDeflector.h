#pragma once

#include "AbstractBeamDeflector.h"

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class representing an oscillating mirror beam deflector
 *
 * Let \f$\theta\f$ be the scan angle, \f$\gamma(p)\f$ be the beam angle on
 *  pulse \f$p\f$, \f$p_t\f$ be the number of pulses required for a turning
 *  operation and \f$n\f$ the number of pulses describing a scacnline.
 *
 * Thus, it is possible to define the first acceleration stage start pulse
 * \f$p_a\f$, the first linear stage start pulse \f$p_b\f$, the first
 *  deceleration stage start pulse \f$p_c\f$, the second acceleration stage
 *  start pulse \f$p_d\f$, the second linear stage start pulse \f$p_e\f$ and
 *  the second deceleration stage start pulse \f$p_f\f$. As follows:
 *
 * \f[
 * \begin{split}
 *  p_a &=&\, 1 \\
 *  p_b &=&\, \left\lfloor\frac{p_t}{2}\right\rfloor \\
 *  p_c &=&\, \left\lfloor\frac{n}{2}\right\rfloor -
 *      \left\lfloor\frac{p_t}{2}\right\rfloor \\
 *  p_d &=&\, \left\lfloor\frac{n}{2}\right\rfloor + 1 \\
 *  p_e &=&\, \left\lfloor\frac{n}{2}\right\rfloor + 1 +
 *      \left\lfloor\frac{p_t}{2}\right\rfloor \\
 *  p_f &=&\, n - \left\lfloor\frac{p_t}{2}\right\rfloor + 1
 * \end{split}
 * \f]
 *
 *
 * Considering the oscillating mirror beam deflector has bitonic scanlines
 *  where, for convenience, the first half of the scanline is the positive
 *  monotonic stage while the second half of the scanline is the negative
 *  monotonic stage. There are \f$6\f$ different stages: The first acceleration
 *  stage inside interval \f$[p_a, p_b)\f$, the first linear stage inside
 *  interval \f$[p_b, p_c)\f$, the first deceleration stage inside interval
 *  \f$[p_c, \frac{n}{2})\f$, the second acceleration stage inside interval
 *  \f$[p_d, p_e)\f$, the second linear stage inside interval
 *  \f$[p_e, p_f)\f$ and the second deceleration stage inside interval
 *  \f$[p_f, n]\f$.
 *
 *
 * Notice that \f$\gamma(0) = -\theta\f$ and
 *  \f$\gamma\left(\frac{n}{2}\right) = \theta\f$ are the frontiers that
 *  must be satisfied.
 *  Then, the oscillating mirror
 *  beam deflector can be modelled as a function \f$\gamma(p)\f$ which gives
 *  the beam angle on pulse \f$p\f$ (as stated before). This function will
 *  behave like the function \f$f\f$ for linear stages, like the function
 *  \f$g\f$ for acceleration stages and like the function \f$h\f$ for
 *  deceleration stages.
 *  These three functions are defined for the first positive monotonic part
 *  of the scanline, without loss of generality (negative monotonic part can
 *  be obtained just by substracting instread of adding):
 * \f[
 * \begin{split}
 *  f(p+1) &=&\, \gamma(p) + \frac{d}{dp}f(p) \\
 *  g(p+1) &=&\, \gamma(p) + \frac{d}{dp}g(p) \\
 *  h(p+1) &=&\, \gamma(p) + \frac{d}{dp}h(p)
 * \end{split}
 * \f]
 *
 *
 * Considering the step between pulses is \f$1\f$, the derivatives with respect
 *  to simulation steps of the previous functions can be defined using forward
 *  finite differences as follows. Where \f$a\f$ and \f$b\f$ denote the first
 *  and last pulse of the current acceleration/linear/deceleration stage,
 *  respectively:
 *
 * \f[
 * \begin{split}
 *  \frac{d}{dp}f(p) \approx&\, \frac{\Delta^{1}_{1}[f](p)}{1} =&\,
 *      \frac{f(p+1)-f(p)}{1} =&\,
 *      \omega \\
 *  \frac{d}{dp}g(p) \approx&\, \frac{\Delta^{1}_{1}[g](p)}{1} =&\,
 *      \frac{g(p+1)-g(p)}{1} =&\,
 *      \frac{p+1-a}{b-a} \omega \\
 *  \frac{d}{dp}h(p) \approx&\, \frac{\Delta^{1}_1[h](p)}{1} =&\,
 *      \frac{h(p+1)-h(p)}{1} =&\,
 *      \frac{b-p-1}{b-a} \omega
 * \end{split}
 * \f]
 *
 *
 * Finally, notice that solving this model reduces to finding the angle between
 *  pulses \f$\omega\f$ that makes the model satisfy its frontiers. It is,
 *  solving below expression for \f$\omega\f$:
 *
 * \f[
 *  \theta = \sum_{p=p_a}^{p_b-1}{\frac{p-p_a}{p_b-p_a}\omega} +
 *      \sum_{p=p_b}^{p_c-1}{\omega} +
 *      \sum_{p=p_c}^{p_d-1}{\frac{p_d-p}{p_d-p_c}\omega} -
 *      \theta
 *  \iff \omega = 2\theta \left[
 *      \sum_{p=p_a}^{p_b-1}{\frac{p-p_a}{p_b-p_a}} +
 *      \left(p_c - p_b\right) +
 *      \sum_{p=p_c}^{p_d-1}{\frac{p_d-p}{p_d-p_c}}
 *  \right]^{-1}
 * \f]
 *
 *
 * Simplifying, \f$\omega\f$ can be simply calculated as depicted below:
 * \f[
 *  \omega = 2 \theta \left(
 *      p_c - p_b + \frac{
 *          p_b - p_a + p_d - p_c - 1
 *      }{2}
 *  \right)^{-1} =
 *  4 \theta \left(
 *      {p_c+p_d-p_a-p_b-1}
 *  \right)^{-1}
 * \f]
 *
 * @see AbstractBeamDeflector
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
    /**
     * @brief The norm to be applied for acceleration and deceleration when
     *  at the middle stage of the scanline. It is, for the first deceleration
     *  and the second acceleration.
     */
    double cached_middleNorm;
    /**
     * @brief The norm to be applied for acceleration and deceleration when
     *  at the extremes of the scanline (its start and its end). It is, for the
     *  first acceleration and the second deceleration.
     */
    double cached_extremeNorm;


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
    /**
     * @see AbstractBeamDeflector::getOpticsType
     */
    std::string getOpticsType() const override {
        return "OSCILLATING_MIRROR";
    }

	/**
	 * @see AbstractBeamDeflector::restartDeflector
	 */
    void restartDeflector() override;

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
	 * @param sign Specify whether the beam angle is increasing (positive, 1)
	 *  or decreasing (negative, -1)
	 * @return
	 */
	void accelerateBeamAngle(
	    double const p,
	    double const pa,
	    double const norm,
        double const sign
    );
	/**
	 * @brief Compute the beam angle linear behavior
	 * @param sign Specify whether the beam angle is increasing (positive, 1)
	 *  or decreasing (negative, -1)
	 */
	void linearBeamAngle(double const sign);
	/**
	 * @brief Compute the beam angle deceleration
	 * @param p Current pulse
	 * @param pb End pulse of the deceleration stage (exclusive)
	 * @param sign Specify whether the beam angle is increasing (positive, 1)
	 *  or decreasing (negative, -1)
	 */
	void decelerateBeamAngle(
	    double const p,
	    double const pb,
        double const norm,
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