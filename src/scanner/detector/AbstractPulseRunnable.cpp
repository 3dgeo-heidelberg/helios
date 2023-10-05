#include "AbstractPulseRunnable.h"

#define _USE_MATH_DEFINES
#include "math.h"
#include "MathConstants.h"
#include <maths/EnergyMaths.h>

#include "AbstractDetector.h"
#include <filems/facade/FMSFacade.h>

#include <glm/glm.hpp>

#include <memory>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
AbstractPulseRunnable::AbstractPulseRunnable(
    std::shared_ptr<Scanner> scanner,
    SimulatedPulse const &pulse
) :
    scanner(scanner),
    pulse(pulse),
    scene(*(scanner->platform->scene))
{
    // Assign corresponding measurement error function
    AbstractDetector &ad = *scanner->getDetector();
    if(ad.errorDistanceExpr != nullptr){
        applyMeasurementError = [&](
            RandomnessGenerator<double> &rg,
            double &distance,
            glm::dvec3 &beamOrigin,
            glm::dvec3 &beamDirection
        ) -> void {
            applyMeasurementErrorFromExpr(
                rg, distance, beamOrigin, beamDirection
            );
        };
    }
    else {
        applyMeasurementError = [&](
            RandomnessGenerator<double> &rg,
            double &distance,
            glm::dvec3 &beamOrigin,
            glm::dvec3 &beamDirection
        ) -> void {
            applyMeasurementErrorDirectly(
                rg, distance, beamOrigin, beamDirection
            );
        };
    }
}

// ***  M E T H O D S  *** //
// *********************** //
void AbstractPulseRunnable::initialize(){
    detector = scanner->getDetector(pulse.getDeviceIndex());
}
void AbstractPulseRunnable::capturePoint(
    Measurement & m,
    RandomnessGenerator<double> &rg,
    std::vector<Measurement> *allMeasurements,
    std::mutex *allMeasurementsMutex,
    std::vector<Measurement> *cycleMeasurements,
    std::mutex *cycleMeasurementsMutex
#if DATA_ANALYTICS >= 2
   ,std::vector<double> &calcIntensityRecord,
   std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
) {
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
#if DATA_ANALYTICS >= 2
    calcIntensityRecord[10] = 1;
    pulseRecorder->recordIntensityCalculation(calcIntensityRecord);
#endif
    if(allMeasurements != nullptr){
        std::unique_lock<std::mutex> lock(*allMeasurementsMutex);
        allMeasurements->push_back(m);
        (allMeasurements->end() - 1)->position += scene.getShift();
    }
    if(cycleMeasurements != nullptr){
        std::unique_lock<std::mutex> lock(*cycleMeasurementsMutex);
        cycleMeasurements->push_back(m);
        (cycleMeasurements->end() - 1)->position += scene.getShift();
    }
    if(detector->pcloudYielder != nullptr) detector->pcloudYielder->push(m);
}

void AbstractPulseRunnable::applyMeasurementErrorDirectly(
    RandomnessGenerator<double> &rg,
    double &distance,
    glm::dvec3 &beamOrigin,
    glm::dvec3 &beamDirection
){
    distance += rg.normalDistributionNext();
}
void AbstractPulseRunnable::applyMeasurementErrorFromExpr(
    RandomnessGenerator<double> &rg,
    double &distance,
    glm::dvec3 &beamOrigin,
    glm::dvec3 &beamDirection
){
    distance = distance + rg.normalDistributionNext()*
        detector->errorDistanceExpr->eval(distance);
}
