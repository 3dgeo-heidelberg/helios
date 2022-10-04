#pragma once

#include <util/HeliosException.h>
#include "AbstractDetector.h"
#include "AbstractPulseRunnable.h"
#include "FullWaveformPulseDetector.h"
#include "RaySceneIntersection.h"
#include "ScenePart.h"
#include <noise/RandomnessGenerator.h>

#include <LadLut.h>

#include <vector>

/**
 * @brief Concrete implementation of abstract pulse runnable to compute full
 * waveform pulses
 *
 * @see AbstractPulseRunnable
 */
class FullWaveformPulseRunnable : public AbstractPulseRunnable {
public:
    // ***  CONSTANTS  *** //
    // ******************* //
    /**
     * @brief Decimal precision constant for FullWaveformPulseRunnable
     * computations
     */
    static const double eps;
private:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Full wavevform pulse detector used to handle pulse computation
     */
	std::shared_ptr<FullWaveformPulseDetector> fwDetector;
	/**
	 * @brief Vector of all measurements. It can be nullptr, since tracking
	 * historical of all measurements might not be requested
	 */
    std::vector<Measurement> * allMeasurements = nullptr;
    /**
     * @brief Mutex to handle concurrent access to vector of all measurements
     */
    std::mutex * allMeasurementsMutex = nullptr;
    /**
     * @brief Vector of current cycle measurements. It can be nullptr, since
     * tracking current cycle measurements might noit be requested
     */
    std::vector<Measurement> * cycleMeasurements = nullptr;
    /**
     * @brief Mutex to handle concurrent access to vector of current cycle
     * measurements
     */
    std::mutex * cycleMeasurementsMutex = nullptr;
    /**
     * @brief Flag to specify if write waveform (true) or not (false)
     */
    bool writeWaveform;
    /**
     * @brief Flag to specify if calc echo width (true) or not (false)
     */
    bool calcEchowidth;
    /**
     * @brief Reference to the scene that is being scanned
     */
    Scene &scene;

public:
    /**
     * @brief Which leg the FullWaveformPulseRunnable belongs to.
     *
     * While this attribute is not strictly necessary for the
     * FullWaveformPulseRunnable to do its job, it really helps with
     * tracing and debugging concurrency issues.
     *
     * For instance, to track what is going on with end of leg
     * FullWaveformPulseRunnable threads while a new leg is being started.
     *
     * This attribute could be safely removed without degenerating class
     * mechanics. So, if in the future it is not wanted any more, feel free
     * to remove it.
     */
    unsigned int legIndex = 0;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Base constructor for full waveform pulse runnable
     * @see AbstractPulseRunnable::AbstractPulseRunnable(
     *  std::shared_ptr<AbstractDetector>, glm::dvec3, Rotation, int, long)
     * @see FullWaveformPulseRunnable::writeWaveform
     * @see FullWaveformPulseRunnable::calcEchowidth
     * @see FullWaveformPulseRunnable::allMeasurements
     * @see FullWaveformPulseRunnable::allMeasurementsMutex
     * @see FullWaveformPulseRunnable::cycleMeasurements
     * @see FullWaveformPulseRunnable::cycleMeasurementsMutex
     * @see FullWaveformPulseRunnable::legIndex
     */
    FullWaveformPulseRunnable(
        std::shared_ptr<FullWaveformPulseDetector> detector,
        glm::dvec3 const absoluteBeamOrigin,
        Rotation const absoluteBeamAttitude,
        int const currentPulseNum,
        double const currentGpsTime,
        bool const writeWaveform,
        bool const calcEchowidth,
        std::vector<Measurement> * allMeasurements,
        std::mutex * allMeasurementsMutex,
        std::vector<Measurement> * cycleMeasurements,
        std::mutex * cycleMeasurementsMutex,
        unsigned int const legIndex
    ) :
        AbstractPulseRunnable(
			detector,
			absoluteBeamOrigin, 
			absoluteBeamAttitude, 
			currentPulseNum, 
			currentGpsTime
        ),
        scene(*(detector->scanner->platform->scene))
	{
		fwDetector = detector;
		this->writeWaveform = writeWaveform;
		this->calcEchowidth = calcEchowidth;
		this->allMeasurements = allMeasurements;
		this->allMeasurementsMutex = allMeasurementsMutex;
		this->cycleMeasurements = cycleMeasurements;
		this->cycleMeasurementsMutex = cycleMeasurementsMutex;
		this->legIndex = legIndex;
	}

	virtual ~FullWaveformPulseRunnable(){}

private:
    // ***  OPERATOR METHODS  *** //
    // ************************** //
    /**
     * @brief Perform ray casting to find intersections
     * @param[in] tMinMax Minimum and maximum time to intersection with respect
     *  to the axis aligned bounding box that bounds the scene
     * @param[out] reflections Where reflections must be stored when a hit is
     *  registered
     * @param[out] intersects Where intersections must be stored when a hit is
     *  registered
     * @see FullWaveformPulseRunnable::handleSubray
     */
    void computeSubrays(
        vector<double> const &tMinMax,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects
    );
    /**
     * @brief Handle sub-rays along the circle
     * @param[in] tMinMax Minimum and maximum time to intersection with respect
     *  to the axis aligned bounding box that bounds the scene
     * @param[in] circleStep The iteration along the circle
     * @param[in] circleStep_rad Angle in radians corresponding to the
     *  iteration
     * @param[in] r1 Sub-beam rotation into divergence step
     * @param[in] divergenceAngle Subray divergence angle in radians
     * @see FullWaveformPulseRunnable::computeSubrays
     */
    void handleSubray(
        vector<double> const &tMinMax,
        int const circleStep,
        double const circleStep_rad,
        Rotation &r1,
        double const divergenceAngle,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects
    );
    /**
     * @brief Digest intersections found through ray casting
     * @param[in] beamDir Beam normalized director vector
     * @param[out] reflections Where reflections must be stored when a hit is
     *  registered
     * @param[out] intersects Where intersections must be stored when a hit is
     *  registered
     * @see FullWaveformPulseRunnable::computeSubrays
     * @see FullWaveformPulseRunnable::findMaxMinHitDistances
     * @see FullWaveformPulseRunnable::initializeFullWaveform
     * @see FullWaveformPulseRunnable::populateFullWaveform
     * @see FullWaveformPulseRunnable::digestFullWaveform
     * @see FullWaveformPulseRunnable::exportOutput
     */
    void digestIntersections(
        std::vector<std::vector<double>>& apMatrix,
        RandomnessGenerator<double> &randGen,
        RandomnessGenerator<double> &randGen2,
        glm::dvec3 &beamDir,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects
    );
    /**
     * @brief Find min and max hit distances in meters
     * @param[out] minHitDist_m Min hit distance will be stored here
     * @param[out] maxHitDist_m Max hit distance will be stored here
     * @see FullWaveformPulseRunnable::digestIntersections
     */
    void findMaxMinHitDistances(
        std::map<double, double> &reflections,
        double &minHitDist_m,
        double &maxHitDist_m
    );
    /**
     * @brief Initialize full waveform
     * While the vector is not strictly initialized in this function,
     * necessary variables are computed here.
     * @param[out] nsPerBin The size of each bin in nano seconds
     * @param[out] distanceThreshold Limit distance threshold
     * @param[out] peakIntensityIndex Index of intensity peak
     * @param[out] numFullwaveBins How many bins are necessary to discretize
     *  the full waveform
     * @return True if it is possible to initialize the full waveform,
     * false otherwise.
     * @see FullWaveformPulseRunnable::digestIntersections
     */
    bool initializeFullWaveform(
        double minHitDist_m,
        double maxHitDist_m,
        double &minHitTime_ns,
        double &maxHitTime_ns,
        double &nsPerBin,
        double &distanceThreshold,
        int &peakIntensityIndex,
        int &numFullwaveBins
    );
    /**
     * @brief Populate a previously initialized full waveform vector
     * @param[out] fullwave Full waveform vector to be populated
     * @see FullWaveformPulseRunnable::digestIntersections
     * @see FullWaveformPulseRunnable::initializeFullWaveform
     */
    void populateFullWaveform(
        std::map<double, double> const &reflections,
        std::vector<double> &fullwave,
        double const distanceThreshold,
        double const minHitTime_ns,
        double const nsPerBin,
        int const peakIntensityIndex
    );
    /**
     * @brief Digest a previously populated full waveform vector,
     * generating measurements
     * @param[out] pointsMeasurement Where generated measurements will be
     * stored
     * @param[out] numReturns Number of returns will be stored here
     * @param fullwave Full waveform vector to be digested
     * @see FullWaveformPulseRunnable::digestIntersections
     * @see FullWaveformPulseRunnable::populateFullWaveform
     * @see Measurement
     */
    void digestFullWaveform(
        std::vector<Measurement> &pointsMeasurement,
        int &numReturns,
        std::vector<std::vector<double>>& apMatrix,
        std::vector<double> const &fullwave,
        vector<RaySceneIntersection> const &intersects,
        glm::dvec3 const &beamDir,
        double const nsPerBin,
        int const numFullwaveBins,
        int const peakIntensityIndex,
        double const minHitTime_ns
    );
    /**
     * @brief Export measurements and full waveform data
     * @param[in] fullwave Full waveform data to export
     * @param[in] pointsMeasurement Point cloud data to export
     * @see FullWaveformPulseRunnable::digestIntersections
     */
    void exportOutput(
        std::vector<double> &fullwave,
        int &numReturns,
        std::vector<Measurement> &pointsMeasurement,
        glm::dvec3 &beamDir,
        double minHitTime_ns,
        double maxHitTime_ns,
        RandomnessGenerator<double> &randGen,
        RandomnessGenerator<double> &randGen2
    );

    // ***  ASSISTANCE METHODS  *** //
    // **************************** //
    /**
     * @brief Find the intersection between the scene and given ray, if any
     * @param[in] tMinMax Minimum and maximum time to intersection with respect
     *  to the axis aligned bounding box that bounds the scene
     * @param o The ray origin
     * @param v The ray director vector
     * @return Intersection between the scene and given ray
     */
    virtual shared_ptr<RaySceneIntersection> findIntersection(
        vector<double> const &tMinMax,
        glm::dvec3 const &o,
        glm::dvec3 const &v
    ) const;
    /**
     * @brief Detect full waveform peaks
     */
    bool detectPeak(
        int const i,
        int const win_size,
        vector<double> const &fullwave
    );

    /**
     * @brief Compute the space distribution equation to calculate the beam
     * energy decreasing the further away from the center
     */
	double calcEmmitedPower(
	    double const radius,
        double const targetRange
    ) const ;
	/**
	 * @brief Capture full wave
	 * @param fullwave Full wave vector
	 * @param fullwaveIndex Full wave index
	 * @param min_time Minimum hit time (nanoseconds)
	 * @param max_time Maximum hit time (nanoseconds)
	 * @param beamOrigin Beam origin in absolute coordinates
	 * @param beamDir Beam director vector
	 * @param gpstime Current GPS time (nanoseconds)
	 * @param fullWaveNoise flag to specify if noise must be added to the
	 *  full wave (true) or not (false)
	 * @param rg2 Randomness generator to be used to add noise to the full wave
	 *  if requested
	 */
	void captureFullWave(
	    std::vector<double> & fullwave,
	    int const fullwaveIndex,
	    double const min_time,
	    double const max_time,
	    glm::dvec3 const &beamOrigin,
	    glm::dvec3 const &beamDir,
        double const gpstime,
	    bool const fullWaveNoise,
	    RandomnessGenerator<double> &rg2
    );

public:
    /**
     * @brief Compute intensity. It is, the strength of the laser going back
     *  to the detector considering the emmited power as computed by
     *  FullWaveformPulseRannaable::calcEmmitedPower
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
	 * @brief Compute intensity through scaling
	 *  AbstractPulseRunnable::calcReceivedPower, which is computed considering
	 *  FullWaveformPulseRunnable::calcEmmitedPower
	 */
	double calcIntensity(
        double const targetRange,
        double const radius,
        double const sigma
    ) const;

	// ***  O P E R A T O R  *** //
	// ************************* //
	/**
	 * @brief Full waveform pulse runnable void functor. It is necessary
	 * due to compatibility reasons.
	 * @see AbstractPulseRunnable::operator()
	 */
	void operator()() override {
	    throw HeliosException(
	        "FullWaveformPulseRunnable operator()() must not be used"
        );
	}
	/**
	 * @brief Full waveform pulse runnable functor
	 * @param apMatrix Reference to matrix to be used to compute Marquardt
	 * fitter
	 * @param randGen A randomness generator
	 * @param randGen2 Another randomness generator
	 * @param intersectionHandlingNoiseSource Noise source to be used at
	 * intersection handling if necessary
	 * @see AbstractPulseRunnable::operator()()
	 */
	void operator() (
	    std::vector<std::vector<double>>& apMatrix,
	    RandomnessGenerator<double> &randGen,
        RandomnessGenerator<double> &randGen2,
        NoiseSource<double> &intersectionHandlingNoiseSource
    ) override;
};