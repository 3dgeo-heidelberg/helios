#pragma once

#include <scanner/detector/PulseTask.h>
#include "AbstractDetector.h"
#include <noise/RandomnessGenerator.h>
class Measurement;
#include "LasSpecification.h"
#include <mutex>


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

    /**
     * @brief The nonnegative integer index of the scanning device.
     *  It is necessary to support scanners with multiple devices / channels
     */
    size_t devIdx;

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
		double const gpsTime,
		size_t const devIdx
	){
		this->detector = detector;
		this->absoluteBeamAttitude = absoluteBeamAttitude;
		this->absoluteBeamOrigin = absoluteBeamOrigin;
		this->currentPulseNum = pulseNumber;
		this->currentGpsTime = gpsTime;
		this->devIdx = devIdx;
	}

	// ***  M E T H O D S  *** //
	// *********************** //
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