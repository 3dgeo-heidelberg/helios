#pragma once

#include <scanner/detector/PulseTask.h>
#include <scanner/detector/AbstractDetector.h>
#include <noise/RandomnessGenerator.h>
class Measurement;
#include "LasSpecification.h"
class Scanner;
#include <scanner/SimulatedPulse.h>


#include <mutex>


/**
 * @brief Base abstract class for pulse runnables
 */
class AbstractPulseRunnable : public PulseTask{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
	 * @brief Scanner used to simulate the pulse
	 */
    std::shared_ptr<Scanner> scanner = nullptr;
    /**
     * @brief Detector used to simulate pulse
     */
	std::shared_ptr<AbstractDetector> detector = nullptr;
	/**
	 * @brief The definition of the pulse to be simulated
	 */
	SimulatedPulse pulse;
    /**
     * @brief Reference to the scene that is being scanned
     */
    Scene &scene;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Base constructor for pulse runnables
	 * @see AbstractPulseRunnable::scanner
	 * @see AbstractPulseRunnable::pulse
	 * @see SimulatedPulse
	 */
	AbstractPulseRunnable(
		std::shared_ptr<Scanner> const scanner,
		SimulatedPulse const &pulse
	);

	// ***  M E T H O D S  *** //
	// *********************** //
	/**
	 * @brief Initialize pending attributes of the abstract pulse runnable
	 *  before doing further computations.
	 *
	 * NOTE that this method alleviates the burden of the sequential thread
	 *  by supporting deferred initialization whenever possible.
	 */
	virtual void initialize();
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