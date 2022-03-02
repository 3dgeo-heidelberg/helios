#include "AbstractPulseRunnable.h"

#define _USE_MATH_DEFINES
#include "math.h"
#include "MathConstants.h"

#include <glm/glm.hpp>

#include "AbstractDetector.h"


// ALS Simplification "Radiometric Calibration..." (Wagner, 2010) Eq. 14
double AbstractPulseRunnable::calcCrossSection(double f, double Alf, double theta) {
	return PI_4 * f * Alf * cos(theta);
}

// Phong reflection model "Normalization of Lidar Intensity..." (Jutzi and Gross, 2009) 
double AbstractPulseRunnable::phongBDRF(
    double incidenceAngle,
    double targetSpecularity,
    double targetSpecularExponent
) {
	double ks = targetSpecularity;
	double kd = (1 - ks);
	double diffuse = kd * cos(incidenceAngle);
	double specularAngle = (incidenceAngle <= PI_HALF) ?
	    incidenceAngle : incidenceAngle - PI_HALF;
	double specular = ks * pow(abs(cos(specularAngle)),targetSpecularExponent);
	return diffuse + specular;
}

// Energy left after attenuation by air particles in range [0,1]
inline double AbstractPulseRunnable::calcAtmosphericFactor(double targetRange){
	return exp(
	    -2 * targetRange * detector->scanner->getAtmosphericExtinction()
    );
}

// Laser radar equation "Signature simulation..." (Carlsson et al., 2000)
double AbstractPulseRunnable::calcReceivedPower(
    double emittedPower,
    double targetRange,
    double incidenceAngle,
    double targetReflectivity,
    double targetSpecularity,
    double targetSpecularExponent,
    double targetArea
) {
	double bdrf = targetReflectivity * phongBDRF(
	    incidenceAngle,
	    targetSpecularity,
	    targetSpecularExponent
    );
	double sigma = calcCrossSection(bdrf, targetArea, incidenceAngle);
    return AbstractPulseRunnable::_calcReceivedPower(
        emittedPower,
        detector->scanner->getDr2(),
        targetRange,
        detector->scanner->getBt2(),
        detector->scanner->getEfficiency(),
        calcAtmosphericFactor(targetRange),
        sigma
    );
}

double AbstractPulseRunnable::calcReceivedPower(
    double emittedPower,
    double targetRange,
    double sigma
){
    return AbstractPulseRunnable::_calcReceivedPower(
        emittedPower,
        detector->scanner->getDr2(),
        targetRange,
        detector->scanner->getBt2(),
        detector->scanner->getEfficiency(),
        calcAtmosphericFactor(targetRange),
        sigma
    );
}

inline double AbstractPulseRunnable::_calcReceivedPower(
    double Pt,
    double Dr2,
    double R,
    double Bt2,
    double etaSys,
    double etaAtm,
    double sigma
){
    return (Pt * Dr2) / (PI_4 * pow(R, 4) * Bt2) * etaSys * etaAtm * sigma;
}

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
    detector->fms->write.writeMeasurement(m);
	detector->mBuffer->add(m);
}

void AbstractPulseRunnable::applyMeasurementError(
    RandomnessGenerator<double> &rg,
    double &distance,
    glm::dvec3 &beamOrigin,
    glm::dvec3 &beamDirection
){
    distance += rg.normalDistributionNext();
}

