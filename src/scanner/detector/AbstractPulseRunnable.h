#pragma once

#include <scanner/detector/PulseTask.h>
#include "AbstractDetector.h"
#include <noise/RandomnessGenerator.h>
class Measurement;
#include "LasSpecification.h"
#include <mutex>

// ############## BEGIN Static variables ###############
/**
 * @brief Speed of light in meters per second
 */
static const double speedOfLight_mPerSec = 299792458;

/**
 * @brief Speed of light in meters per nanosecond
 */
static const double cfg_speedOfLight_mPerNanosec = 0.299792458;

/**
 * @brief Speed of light in meters per picosecond
 */
static const double speedOfLight_mPerPicosec = 0.000299792458;
// ############## END Static variables ###############

/**
 * @brief Base abstract class for pulse runnables
 */
class AbstractPulseRunnable : public PulseTask{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Detector used to simulate pulse
     */
	std::shared_ptr<AbstractDetector> detector = nullptr;


    /**
     * @brief Beam origin in absolute coordinates
     */
	glm::dvec3 absoluteBeamOrigin;
	/**
	 * @brief Beam attitude
	 */
	Rotation absoluteBeamAttitude;

	/**
	 * @brief Number of current pulse
	 */
	int currentPulseNum;
	/**
	 * @brief Current GPS time in nanoseconds
	 */
    double currentGpsTime; // In nanoseconds

	/**
	 * @brief Flag to specify if ground points must be captured (true) or not
	 * (false)
	 */
	bool writeGround = true;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Base constructor for pulse runnables
	 * @see AbstractPulseRunnable::detector
	 * @see AbstractPulseRunnable::absoluteBeamOrigin
	 * @see AbstractPulseRunnable::absoluteBeamAttitude
	 * @see AbstractPulseRunnable::currentPulseNum
	 * @see AbstractPulseRunnable::currentGpsTime
	 */
	AbstractPulseRunnable(
		std::shared_ptr<AbstractDetector> const detector,
		glm::dvec3 const absoluteBeamOrigin,
		Rotation const absoluteBeamAttitude,
		int const pulseNumber,
		double const gpsTime
	){
		this->detector = detector;
		this->absoluteBeamAttitude = absoluteBeamAttitude;
		this->absoluteBeamOrigin = absoluteBeamOrigin;
		this->currentPulseNum = pulseNumber;
		this->currentGpsTime = gpsTime;
	}

	// ***  M E T H O D S  *** //
	// *********************** //
	/**
	 * @brief Compute atmospheric factor \f$f\f$, understood as the energy left
	 * after attenuation by air particles in range \f$[0, 1]\f$
	 * @param targetRange \f$r\f$
	 *
	 * Let \f$A_{e}\f$ be the atmospheric extinction
	 * \f[
	 *  f = e^{-2r \cdot A_{e}}
	 * \f]
	 *
	 * @return \f$f\f$
	 */
	inline double calcAtmosphericFactor(double const targetRange) const;
	/**
	 * @brief Solve the laser radar equation
	 *
	 * <br/>
	 * Report title: Signature simulation and signal analysis for 3-D laser
	 * radar
	 * <br/>
	 * Report authors: Tomas Carlsson, Ove Steinvall and Dietmar Letalick
	 */
	double calcReceivedPower(
		double const emittedPower,
		double const targetRange,
		double const incidenceAngle,
		double const targetReflectivity,
		double const targetSpecularity,
		double const targetSpecularExponent,
		double const targetArea
	) const;
	/**
	 * @brief Alternative received power computation method
	 *
	 * @param emittedPower Emitted power
	 * @param targetRange The distance with respect to intersection
	 * multiplied by the sine of the divergence angle
	 * @param sigma Sigma value taken from LadLut specification
	 *
	 * @return Received power
	 *
	 * @see LadLut
	 * @see AbstractPulseRunnable::_calcReceivedPower
	 */
	double calcReceivedPower(
	    double const emittedPower,
	    double const targetRange,
	    double const sigma
    ) const ;
	/**
	 * @brief Compute received power \f$P_{r}\f$
	 *
	 * \f[
	 *  P_{r} = \textrm{etaSys} \cdot \textrm{etaAm} \cdot \sigma \cdot
	 *      \frac{P_{t} \cdot D_{r2}}{4{\pi} \cdot R^{4} \cdot B_{t2}}
	 * \f]
	 */
	static inline double _calcReceivedPower(
	    double const Pt,
	    double const Dr2,
	    double const R,
	    double const Bt2,
	    double const etaSys,
	    double const etaAtm,
	    double const sigma
    );
	/**
	 * @brief Capture point if proceed and write it
	 * @param m Measurement
	 * @param rg Randomness generator
	 * @param allMeasurements Vector of all measurements to store captured
	 * point if requested
	 * @param allMeasurementsMutex Mutex to handle concurrent access to
	 * vector of all measurements
	 * @param cycleMeasurements Vector of current cycle measurements to store
	 * captured point if requested
	 * @param cycleMeasurementsMutex Mutex to handle concurrent access to
	 * vector of current cycle measurements
	 */
	void capturePoint(
	    Measurement & m,
	    RandomnessGenerator<double> &rg,
	    std::vector<Measurement> *allMeasurements,
        std::mutex *allMeasurementsMutex,
        std::vector<Measurement> *cycleMeasurements,
        std::mutex *cycleMeasurementsMutex
    );
	/**
	 * @brief Apply error to received measurement
	 *
	 * @param rg RandomnessGenerator to be used to apply error to the measure
	 * @param distance Reference to the distance where error shall be applied
	 * @param beamOrigin Reference to the beam originWaypoint where error shall be
	 * applied
	 * @bream beamDirection Reference to the beam direction where error shall
	 * be applied
	 */
	void applyMeasurementError(
	    RandomnessGenerator<double> &rg,
	    double &distance,
	    glm::dvec3 &beamOrigin,
	    glm::dvec3 &beamDirection
    );

};