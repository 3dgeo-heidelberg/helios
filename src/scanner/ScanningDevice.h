#pragma once

#include <maths/Rotation.h>
#include <maths/Directions.h>
#include <assetloading/Asset.h>
#include <scanner/ScannerHead.h>
#include <scanner/FWFSettings.h>
#include <scanner/beamDeflector/AbstractBeamDeflector.h>
class AbstractDetector;
#include <scene/RaySceneIntersection.h>
#include <noise/NoiseSource.h>

#include <glm/glm.hpp>

#include <string>
#include <list>
#include <unordered_map>
#include <map>

class Scanner;
class SingleScanner;
class MultiScanner;


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a scanning device. Any scanner needs to be
 *  composed of at least one scanning device to be computable.
 */
class ScanningDevice : public Asset {
protected:
    // ***  F R I E N D S  *** //
    // *********************** //
    friend class Scanner;
    friend class SingleScanner;
    friend class MultiScanner;

    // ***  DEVICE ATTRIBUTES  *** //
    // *************************** //
    /**
	 * @brief Device identifier
	 */
    std::string id = "";
    /**
     * @brief Head relative emitter position
     */
    glm::dvec3 headRelativeEmitterPosition = glm::dvec3(0, 0, 0);
    /**
	 * @brief Head relative emitter attitude
	 */
    Rotation headRelativeEmitterAttitude =
        Rotation(Directions::right, 0);
    /**
     * @brief Beam divergence (radians)
     */
    double beamDivergence_rad = 0;
    /**
	 * @brief Pulse length (nanoseconds)
	 */
    double pulseLength_ns = 0;
    /**
	 * @brief Average power (watts)
	 */
    double averagePower_w;
    /**
	 * @brief Beam quality
	 */
    double beamQuality;
    /**
	 * @brief Device efficiency
	 */
    double efficiency;
    /**
	 * @brief Receiver diameter (meters)
	 */
    double receiverDiameter_m;
    /**
	 * @brief Visibility (kilometers)
	 */
    double visibility_km;
    /**
	 * @brief Wave length (meters)
	 */
    double wavelength_m;
    /**
	 * @brief Atmospheric extinction
	 */
    double atmosphericExtinction;
    /**
	 * @brief Beam waist radius
	 */
    double beamWaistRadius;
    /**
     * @brief Scanner head composing the scanning device
     * @see ScannerHead
     */
    std::shared_ptr<ScannerHead> scannerHead;
    /**
	 * @brief Beam deflector composing the scanner
	 * @see AbstractBeamDeflector
	 */
    std::shared_ptr<AbstractBeamDeflector> beamDeflector;
    /**
	 * @brief Detector composing the scanner
	 * @see AbstractDetector
	 */
    std::shared_ptr<AbstractDetector> detector;
    /**
	 * @brief Full wave form settings for the scanner
	 * @see FWFSettings
	 */
    FWFSettings FWF_settings;
    /**
     * @brief Number of rays computed by the calcRaysNumber function
     * @see ScanningDevice::calcRaysNumber
     */
    int numRays = 0;
    /**
	 * @brief Pulse frequencies (hertz) supoported by the scanner
	 */
    std::list<int> supportedPulseFreqs_Hz;
    /**
     * @brief Maximum number of returns per pulse. When 0, it means there is
     *  not maximum at all
     * @see Scanner::checkMaxNOR
     */
    int maxNOR = 0;
    /**
	 * @brief Number of bins defining the discretization size
	 *
	 * The number of bins is computed considering full wave settings:
	 * \f[
	 *  \textrm{numTimeBins} = \frac{\textrm{pulseLength}}{binSize}
	 * \f]
	 */
    int numTimeBins = -1;
    /**
	 * @brief Index of bin containing the intensity peak. It is computed
	 * through calcTimePropagation function.
	 *
	 * @see Scanner::calcTimePropagation(vector<double> &, int)
	 */
    int peakIntensityIndex = -1;
    /**
     * @brief Discretization vector
     */
    std::vector<double> time_wave;

    // ***  CACHED ATTRIBUTES  *** //
    // *************************** //
    /**
	 * @brief \f$D_{r2}\f$ understood as the square of receiver diameter
	 *
	 * \f[
	 *  D_{r2} = \textrm{receiverDiamater}^{2}
	 * \f]
	 *
	 * @see ScanningDevice::receiverDiameter_m
	 */
    double cached_Dr2;
    /**
	 * @brief \f$B_{t2}\f$ understood as the square of beam divergence
	 *
	 * \f[
	 *  B_{t2} = \textrm{beamDivergence}^{2}
	 * \f]
	 *
	 * @see ScanningDevice::beamDivergence_rad
	 */
    double cached_Bt2;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief ScanningDevice constructor from given values
     */
    ScanningDevice(
        std::string const id,
        double const beamDiv_rad,
        glm::dvec3 const beamOrigin,
        Rotation const beamOrientation,
        std::list<int> const &pulseFreqs,
        double const pulseLength_ns,
        double const averagePower,
        double const bemQuality,
        double const efficiency,
        double const receiverDiameter_m,
        double const atmosphericVisibility_km,
        double const wavelength_m
    );
    /**
     * @brief Copy constructor for the ScanningDevice
     * @param scdev The scanning device to be copied
     */
    ScanningDevice(ScanningDevice &scdev);
    virtual ~ScanningDevice() = default;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Configure beam related attributes. It is recommended to
     *  reconfigure beam attributes always that beam divergence, beam quality
     *  or wavelength are updated.
     * @see ScanningDevice::beamDivergence_rad
     * @see ScanningDevice::beamQuality
     * @see ScanningDevice::wavelength_m
     * @see ScanningDevice::beamWaistRadius
     * @see ScanningDevice::cached_Bt2
     */
    void configureBeam();
    /**
     * @brief Compute the atmospheric attenuation to be used as the
     *  atmospheric extinction
     * @return Atmospheric attenuation
     * @see Scanner::atmosphericExtinction
     */
    double calcAtmosphericAttenuation() const;
    /**
     * @brief Compute the number of rays depending on beam sample quality
     */
    void calcRaysNumber();
    /**
     * @brief Do the simulation steps of the scanning device
     * @param legIndex Current simulation leg index
     * @param currentGpsTime Current simulated GPS time
     * @param simFreq_Hz The simulation frequency in Hertz
     * @param isActive True if the scanner is active, False otherwise
     * @param platformPosition The absolute mount position of the
     *  platform
     */
    void doSimStep(
        unsigned int legIndex,
        double const currentGpsTime,
        int const simFreq_Hz,
        bool const isActive,
        glm::dvec3 const &platformPosition,
        Rotation const &platformAttitude,
        std::function<void(glm::dvec3 &, Rotation &)> handleSimStepNoise,
        std::function<void(
            unsigned int, glm::dvec3 &, Rotation &, double const
        )> handlePulseComputation
    );
    /**
     * @brief Compute the absolute beam attitude of the scanning device
     *  with respect to given absolute platform attitude
     * @þaram platformAttitude The absolute mount attitude of the platform
     *  where the scanning device is placed
     * @return The absolute beam attitude of the scanning device with respect
     *  to given absolute platform attitude
     */
    Rotation calcAbsoluteBeamAttitude(Rotation platformAttitude);

    /**
     * @see Scanner::computeSubrays
     */
    void computeSubrays(
        std::function<void(
            std::vector<double> const &_tMinMax,
            int const circleStep,
            double const circleStep_rad,
            Rotation &r1,
            double const divergenceAngle,
            NoiseSource<double> &intersectionHandlingNoiseSource,
            std::map<double, double> &reflections,
            vector<RaySceneIntersection> &intersects
        )> handleSubray,
        std::vector<double> const &tMinMax,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        std::vector<RaySceneIntersection> &intersects
    );
    /**
     * @see Scanner::initializeFullWaveform
     */
    bool initializeFullWaveform(
        double const minHitDist_m,
        double const maxHitDist_m,
        double &minHitTime_ns,
        double &maxHitTime_ns,
        double &nsPerBin,
        double &distanceThreshold,
        int &peakIntensityIndex,
        int &numFullwaveBins
    );

    /**
     * @brief Compute intensity. It is the strength of the laser going back
     *  to the detector considering the emitted power \f$P_e\f$, and its
     *  corresponding received power \f$P_r\f$.
     *
     * Let \f$I_0\f$ be the average power of the scanning device, \f$\lambda\f$
     *  be the wavelength, \f$R\f$ be the target range, \f$R_0\f$ be the
     *  minimum range, and \f$w_0\f$ be the beam waist radius. Thus, the
     *  emitted power \f$P_e\f$ can be defined:
     * \f[
     *  P_e = I_0 \exp\left[- \frac{
     *          2 \pi^2 r^2 w_0^2
     *      }{
     *          \lambda^2 \left(R_0^2 + R^2\right)
     *      }\right]
     * \f]
     *
     * Note that if \f$a_e\f$ is the atmospheric extinction coefficient, then
     *  the atmospheric factor \f$\eta_a\f$ is:
     * \f[
     *  \eta_a = \exp\left(-2Ra_e\right)
     * \f]
     *
     * Now, let \f$\eta_s\f$ be the efficiency of the scanning device,
     *  \f$\sigma\f$ be the cross section between the target area and the
     *  incidence angle, \f$D_{r2}\f$ the squared receiver diameter, and
     *  \f$B_{t2}\f$ the squared beam divergence. Thus, the received power
     *  can be calculated as:
     * \f[
     * \begin{split}
     *  P_r =& \frac{P_e D_r^2 \eta_s \eta_a \sigma}{4 \pi R^4 B_t^2} \\
     *   =& \frac{
     *          I_0 D_r^2 \eta_s \sigma
     *      }{
     *          4 \pi R^4 B_t^2
     *      }
     *      \exp\left[-\left(
     *          \frac{2\pi^2r^2w_0^2}{\lambda^2\left(R_0^2 + R^2\right)} +
     *          2Ra_e
     *      \right)\right]
     * \end{split}
     * \f]
     *
     * Finally, the intensity would be \f$P_r 10^9\f$.
     *
     * @return Computed intensity \f$P_r 10^9\f$
     */
    double calcIntensity(
        double const incidenceAngle,
        double const targetRange,
        double const targetReflectivity,
        double const targetSpecularity,
        double const targetSpecularExponent,
        double const targetArea,
        double const radius
    ) const;

    /**
     * @brief Version of ScanningDevice::calcIntensity with precomputed
     *  \f$\sigma\f$
     * @see ScanningDevice::calcIntensity
     */
    double calcIntensity(
        double const targetRange,
        double const radius,
        double const sigma
    ) const;

};