#include "AbstractPulseRunnable.h"

#define _USE_MATH_DEFINES
#include "math.h"
#include "MathConstants.h"
#include <maths/EnergyMaths.h>

#include <glm/glm.hpp>

#include "AbstractDetector.h"
#include <filems/facade/FMSFacade.h>


void AbstractPulseRunnable::capturePoint(
    Measurement & m,
    RandomnessGenerator<double> &rg,
    std::vector<Measurement> *allMeasurements,
    std::mutex *allMeasurementsMutex,
    std::vector<Measurement> *cycleMeasurements,
    std::mutex *cycleMeasurementsMutex
) {
	if (!writeGround && m.classification == LasSpecification::GROUND) {
		return;
	}

	// Abort if point distance is below mininum scanner range:
	// TODO Pending : This check is already done in FullWaveformPulseRunnable
	// What is the point on repeating it?
	if (m.distance < detector->cfg_device_rangeMin_m) {
		return;
	}
	// ########## BEGIN Apply gaussian range accuracy error ###########
	applyMeasurementError(rg, m.distance, m.beamOrigin, m.beamDirection);
	// ########## END Apply gaussian range accuracy error ###########

	// Calculate final recorded point coordinates:
	// TODO Pending : Is it necessary to compute position again? Notice it is
    // known from ray intersection point at FullWaveformPulseRunnable
	m.position = m.beamOrigin + m.beamDirection * m.distance;
    if(allMeasurements != nullptr){
        std::unique_lock<std::mutex> lock(*allMeasurementsMutex);
        allMeasurements->push_back(m);
        (allMeasurements->end() - 1)->position +=
            detector->scanner->platform->scene->getShift();
    }
    if(cycleMeasurements != nullptr){
        std::unique_lock<std::mutex> lock(*cycleMeasurementsMutex);
        cycleMeasurements->push_back(m);
        (cycleMeasurements->end() - 1)->position +=
            detector->scanner->platform->scene->getShift();
    }
    detector->pcloudYielder->push(m);
}

void AbstractPulseRunnable::applyMeasurementError(
    RandomnessGenerator<double> &rg,
    double &distance,
    glm::dvec3 &beamOrigin,
    glm::dvec3 &beamDirection
){
    distance += rg.normalDistributionNext();
}

