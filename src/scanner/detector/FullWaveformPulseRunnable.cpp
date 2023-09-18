// TODO Pending : This implementation is calling scanner setLastPulseWasHit
// Is this thread safe?
#include "FullWaveformPulseRunnable.h"

#include "logging.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "maths/Rotation.h"
#include <maths/EnergyMaths.h>
#include "MarquardtFitter.h"
#include <TimeWatcher.h>
#include <maths/RayUtils.h>
#include <filems/facade/FMSFacade.h>
#include <scanner/detector/FullWaveform.h>
#include <scanner/Scanner.h>

using namespace std;

// ***  CONSTANTS  *** //
// ******************* //
const double FullWaveformPulseRunnable::eps = 0.001;


// ***  O P E R A T O R  *** //
// ************************* //
void FullWaveformPulseRunnable::operator()(
	std::vector<std::vector<double>>& apMatrix,
	RandomnessGenerator<double> &randGen,
	RandomnessGenerator<double> &randGen2,
	NoiseSource<double> &intersectionHandlingNoiseSource
#ifdef DATA_ANALYTICS
    ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    // Deferred/lazy initialization
    initialize();
	// Compute beam direction
	glm::dvec3 beamDir = pulse.computeDirection();

	// NOTE:
	// With beam divergence / full waveform being simulated, this is not perfect, since a sub-ray
	// might hit the scene even if the central ray does now. However, this check is a very important
	// performance optimization, so we should keep it nevertheless. sbecht 2016-04-24

	// Early abort if central axis of the beam does not intersect with the scene:
	vector<double> tMinMax = scene.getAABB()->getRayIntersection(
	    pulse.getOriginRef(),
	    beamDir
    );
	if (tMinMax.empty()) {
		logging::DEBUG("Early abort - beam does not intersect with the scene");
		scanner->setLastPulseWasHit(false, pulse.getDeviceIndex());
		return;
	}

	// Ray casting (find intersections)
	map<double, double> reflections;
	vector<RaySceneIntersection> intersects;
	computeSubrays(
	    tMinMax,
	    intersectionHandlingNoiseSource,
	    reflections,
	    intersects
#ifdef DATA_ANALYTICS
       ,pulseRecorder
#endif
    );

	// Digest intersections
	digestIntersections(
	    apMatrix,
	    randGen,
	    randGen2,
	    beamDir,
	    reflections,
	    intersects
    );
}




// ***  OPERATOR METHODS  *** //
// ************************** //
void FullWaveformPulseRunnable::initialize(){
    AbstractPulseRunnable::initialize();
    fwDetector = std::static_pointer_cast<FullWaveformPulseDetector>(
        detector
    );
}
void FullWaveformPulseRunnable::computeSubrays(
    vector<double> const &tMinMax,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    std::map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
#ifdef DATA_ANALYTICS
    ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    scanner->computeSubrays(
        [&] (
            vector<double> const &_tMinMax,
            int const circleStep,
            double const circleStep_rad,
            Rotation &r1,
            double const divergenceAngle,
            NoiseSource<double> &intersectionHandlingNoiseSource,
            std::map<double, double> &reflections,
            vector<RaySceneIntersection> &intersects
        ) -> void {
            handleSubray(
                _tMinMax,
                circleStep,
                circleStep_rad,
                r1,
                divergenceAngle,
                intersectionHandlingNoiseSource,
                reflections,
                intersects
#ifdef DATA_ANALYTICS
                ,pulseRecorder
#endif
            );
        },
        tMinMax,
        intersectionHandlingNoiseSource,
        reflections,
        intersects,
        pulse.getDeviceIndex()
    );
}

void FullWaveformPulseRunnable::handleSubray(
    vector<double> const &_tMinMax,
    int const circleStep,
    double const circleStep_rad,
    Rotation &r1,
    double const divergenceAngle,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
#ifdef DATA_ANALYTICS
    ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    // Rotate around the circle:
    vector<double> tMinMax = _tMinMax;
    Rotation r2 = Rotation(Directions::forward, circleStep_rad * circleStep);
    r2 = r2.applyTo(r1);

    glm::dvec3 subrayDirection = pulse.getAttitude().applyTo(r2)
        .applyTo(Directions::forward);

    glm::dvec3 subrayOrigin(pulse.getOrigin());
    bool rayContinues = true;
    double incidenceAngle = 0.0;
    while(rayContinues) {
        rayContinues = false;
        shared_ptr<RaySceneIntersection> intersect = findIntersection(
            tMinMax,
            subrayOrigin,
            subrayDirection
        );

        if (intersect != nullptr && intersect->prim != nullptr) {
            // Incidence angle:
            if(!scanner->isFixedIncidenceAngle()) {
                incidenceAngle =
                    intersect->prim->getIncidenceAngle_rad(
                        pulse.getOriginRef(),
                        subrayDirection,
                        intersect->point
                    );
            }

            // Distance between beam origin and intersection:
            double distance = glm::distance(
                intersect->point,
                pulse.getOriginRef()
            );

            // Distance must be inside [rangeMin, rangeMax] interval
            if(detector->isDistanceNotInRange(distance)) continue;

            // Distance between beam's center line and intersection point:
            double const radius = sin(divergenceAngle) * distance;
            double const targetArea = scanner->calcTargetArea(
                distance, pulse.getDeviceIndex()
            );
            double intensity = 0.0;
            if(intersect->prim->canComputeSigmaWithLadLut()){
                // LadLut based intensity computation
                double sigma = intersect->prim->computeSigmaWithLadLut(
                    subrayDirection
                );
                intensity = scanner->calcIntensity(
                    distance, radius, sigma, pulse.getDeviceIndex()
                );
            }
            else{
                // Lighting-based intensity computation
                intensity = scanner->calcIntensity(
                    incidenceAngle,
                    distance,
                    *intersect->prim->material,
                    targetArea,
                    radius,
                    pulse.getDeviceIndex()
#ifdef DATA_ANALYTICS
                   ,pulseRecorder
#endif
                );
            }


            // Intersection handling
            if(intersect->prim->canHandleIntersections()) {
                glm::dvec3 outsideIntersectionPoint =
                    RayUtils::obtainPointAfterTraversing(
                        *intersect->prim->getAABB(),
                        subrayOrigin,
                        subrayDirection,
                        0.0
                    );
                IntersectionHandlingResult ihr =
                    intersect->prim->onRayIntersection(
                        intersectionHandlingNoiseSource,
                        subrayDirection,
                        intersect->point,
                        outsideIntersectionPoint,
                        intensity
                    );
                if (ihr.canRayContinue()) { // Subray can continue
                    // Move subray origin outside primitive and update tMinMax
                    subrayOrigin = outsideIntersectionPoint +
                                   0.00001 * subrayDirection;
                    tMinMax = scene.getAABB()->getRayIntersection(
                        subrayOrigin,
                        subrayDirection
                    );
                    rayContinues = true;
                }
                else{ // Update distance considering noise
                    distance = glm::distance(
                        ihr.getIntersectionPoint(),
                        pulse.getOriginRef()
                    );
                }
            }
            if(!rayContinues) { // If ray is not continuing
                // Then register hit by default
                reflections.insert(
                    pair<double, double>(distance, intensity)
                );
                intersects.push_back(*intersect);
            }
        }
    }
}

void FullWaveformPulseRunnable::digestIntersections(
    std::vector<std::vector<double>>& apMatrix,
    RandomnessGenerator<double> &randGen,
    RandomnessGenerator<double> &randGen2,
    glm::dvec3 &beamDir,
    std::map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
){
    // Find maximum hit distances
    double minHitDist_m, maxHitDist_m;
    findMaxMinHitDistances(reflections, minHitDist_m, maxHitDist_m);

    // If nothing was hit, get out of here
    if (maxHitDist_m < 0) {
        scanner->setLastPulseWasHit(false, pulse.getDeviceIndex());
        return;
    }

    // Initialize full waveform
    double nsPerBin, distanceThreshold, minHitTime_ns, maxHitTime_ns;
    int peakIntensityIndex, numFullwaveBins;
    if(!initializeFullWaveform(
        minHitDist_m,
        maxHitDist_m,
        minHitTime_ns,
        maxHitTime_ns,
        nsPerBin,
        distanceThreshold,
        peakIntensityIndex,
        numFullwaveBins
    )) return;
    std::vector<double> fullwave(numFullwaveBins, 0.0);

    // Populate full waveform
    populateFullWaveform(
        reflections,
        fullwave,
        distanceThreshold,
        minHitTime_ns,
        nsPerBin,
        peakIntensityIndex
    );

    // Digest full waveform, generating measurements
    vector<Measurement> pointsMeasurement;
    int numReturns;
    digestFullWaveform(
        pointsMeasurement,
        numReturns,
        apMatrix,
        fullwave,
        intersects,
        pulse.computeExactDirection(),
        nsPerBin,
        numFullwaveBins,
        peakIntensityIndex,
        minHitTime_ns
    );

    // Export measurements and full waveform data
    exportOutput(
        fullwave,
        numReturns,
        pointsMeasurement,
        beamDir,
        minHitTime_ns,
        maxHitTime_ns,
        randGen,
        randGen2
    );
}

void FullWaveformPulseRunnable::findMaxMinHitDistances(
    std::map<double, double> &reflections,
    double &minHitDist_m,
    double &maxHitDist_m
){
    // Initialize min-max hit distances
    maxHitDist_m = -1.0;
    minHitDist_m = numeric_limits<double>::max();

    // Find min-max hit distances
    map<double, double>::iterator it;
    for (it = reflections.begin(); it != reflections.end(); it++) {
        double const entryDistance = it->first;
        if(entryDistance > maxHitDist_m) {
            maxHitDist_m = entryDistance;
        }
        if(entryDistance < minHitDist_m) {
            minHitDist_m = entryDistance;
        }
    }
}

bool FullWaveformPulseRunnable::initializeFullWaveform(
    double const minHitDist_m,
    double const maxHitDist_m,
    double &minHitTime_ns,
    double &maxHitTime_ns,
    double &nsPerBin,
    double &distanceThreshold,
    int &peakIntensityIndex,
    int &numFullwaveBins
){
    return scanner->initializeFullWaveform(
        minHitDist_m,
        maxHitDist_m,
        minHitTime_ns,
        maxHitTime_ns,
        nsPerBin,
        distanceThreshold,
        peakIntensityIndex,
        numFullwaveBins,
        pulse.getDeviceIndex()
    );
}

void FullWaveformPulseRunnable::populateFullWaveform(
    std::map<double, double> const &reflections,
    std::vector<double> &fullwave,
    double const distanceThreshold,
    double const minHitTime_ns,
    double const nsPerBin,
    int const peakIntensityIndex
){
    // Multiply each sub-beam intensity with time_wave and
    // add to the full waveform
    vector<double> const &time_wave = scanner->getTimeWave(
        pulse.getDeviceIndex()
    );
    map<double, double>::const_iterator it;
    for (it = reflections.begin(); it != reflections.end(); ++it) {
        double const entryDistance_m = it->first;
        if(entryDistance_m > distanceThreshold) continue;
        double const entryIntensity = it->second;
        double const wavePeakTime_ns = entryDistance_m /
            SPEEDofLIGHT_mPerNanosec; // in nanoseconds
        int const binStart = std::max(
            (
                (
                    (int) ((wavePeakTime_ns-minHitTime_ns) / nsPerBin)
                ) - peakIntensityIndex
            ),
            0
        );
        for (size_t i = 0; i < time_wave.size(); ++i) {
            fullwave[binStart + i] += time_wave[i] * entryIntensity;
        }
    }
}

void FullWaveformPulseRunnable::digestFullWaveform(
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
){
    // Extract points from waveform data via Gaussian decomposition
    size_t const devIdx = pulse.getDeviceIndex();
    numReturns = 0;
    int win_size = (int)(
        scanner->getFWFSettings(pulse.getDeviceIndex()).winSize_ns/nsPerBin
    );
    // search for peaks around [-win_size, win_size]

    // least-squares estimation
    MarquardtFitter fit = MarquardtFitter(apMatrix);
    fit.setData(fullwave);

    double echo_width = 0.0;

    for (int i = 0; i < numFullwaveBins; ++i) {
        if (fullwave[i] < eps) continue;

        // Continue with next iteration if there is no peak
        if(!detectPeak(i, win_size, fullwave)) continue;

        // Gaussian model fitting
        if (scanner->isCalcEchowidth()) {
            try {
                fit.setParameters(
                    vector<double>{0, fullwave[i], (double) i, 1}
                );
                fit.fitData();
            }
            catch (std::exception &e) {
                logging::ERR(e.what());
                continue;
            }
            echo_width = fit.getParameters()[3];
            echo_width = echo_width * nsPerBin;

            if (echo_width < 0.1) { // TODO Pending : 0.1 to threshold variable
                continue;
            }
        }

        // Compute distance
        double distance = SPEEDofLIGHT_mPerNanosec *
            (i * nsPerBin + minHitTime_ns);

        // Build list of objects that produced this return
        double minDifference = numeric_limits<double>::max();
        shared_ptr<RaySceneIntersection> closestIntersection = nullptr;

        for (RaySceneIntersection intersect : intersects) {
            double const intersectDist = glm::distance(
                intersect.point, pulse.getOriginRef());
            double const absDistDiff = std::fabs(intersectDist-distance);

            if (absDistDiff < minDifference) {
                minDifference = absDistDiff;
                closestIntersection = make_shared<RaySceneIntersection>(
                    intersect);
            }
        }

        string hitObject;
        int classification = 0;
        if (closestIntersection != nullptr) {
            hitObject = closestIntersection->prim->part->mId;
            if(
                closestIntersection->prim->part->getType() ==
                ScenePart::DYN_MOVING_OBJECT
            ){  // Skip DSMO_ prefix for dynamic moving objects
                hitObject = hitObject.substr(5);
            }
            classification =
                closestIntersection->prim->material->classification;
        }

        // Add distance error (mechanical range error)
        if(pulse.hasMechanicalError()) {
            distance += pulse.getMechanicalRangeError();
        }

        // Build measurement
        Measurement tmp;
        tmp.devIdx = devIdx;
        tmp.devId = scanner->getDeviceId(devIdx);
        tmp.beamOrigin = pulse.getOrigin();
        tmp.beamDirection = beamDir;
        tmp.distance = distance;
        tmp.echo_width = echo_width;
        tmp.intensity = fullwave.at(i);
        tmp.fullwaveIndex = pulse.getPulseNumber();
        tmp.hitObjectId = hitObject;
        tmp.returnNumber = numReturns + 1;
        tmp.classification = classification;
        tmp.gpsTime = pulse.getTime();

        pointsMeasurement.push_back(tmp);
        ++numReturns;

        // Check if maximum number of returns per pulse has been reached
        if(!scanner->checkMaxNOR(numReturns, devIdx)) break;
    }

}

void FullWaveformPulseRunnable::exportOutput(
    std::vector<double> & fullwave,
    int const numReturns,
    std::vector<Measurement> &pointsMeasurement,
    glm::dvec3 const &beamDir,
    double const minHitTime_ns,
    double const maxHitTime_ns,
    RandomnessGenerator<double> &randGen,
    RandomnessGenerator<double> &randGen2
){
    // ############ END Extract points from waveform data ################
    if (numReturns > 0) {
        for (Measurement & pm : pointsMeasurement) {
            pm.pulseReturnNumber = numReturns;
            capturePoint(
                pm,
                randGen,
                scanner->allMeasurements.get(),
                scanner->allMeasurementsMutex.get(),
                scanner->cycleMeasurements.get(),
                scanner->cycleMeasurementsMutex.get()
            );
        }
        if(scanner->isWriteWaveform()) {
            scanner->setLastPulseWasHit(true, pulse.getDeviceIndex());
            captureFullWave(
                fullwave,
                pulse.getPulseNumber(),
                minHitTime_ns,
                maxHitTime_ns,
                pulse.getOrigin() + scene.getShift(),
                beamDir,
                pulse.getTime(),
                scanner->isFullWaveNoise(),
                randGen2
            );
        }
    }
    // ############ END Create full waveform ##############
}




// ***  ASSISTANCE METHODS  *** //
// **************************** //
shared_ptr<RaySceneIntersection> FullWaveformPulseRunnable::findIntersection(
    vector<double> const &tMinMax,
    glm::dvec3 const &o,
    glm::dvec3 const &v
) const {
    return scene.getIntersection(tMinMax, o, v, false);
}

void FullWaveformPulseRunnable::captureFullWave(
    vector<double> & fullwave,
    int const fullwaveIndex,
    double const min_time,
    double const max_time,
    glm::dvec3 const &beamOrigin,
    glm::dvec3 const &beamDir,
    double const gpstime,
    bool const fullWaveNoise,
    RandomnessGenerator<double> &rg2
){
    // Add noise to the fullwave
    if(fullWaveNoise) {
        double const halfDevAcc = detector->cfg_device_accuracy_m / 2.0;
        for (double &fw : fullwave) {
            fw += rg2.normalDistributionNext() * halfDevAcc;
        }
    }

    // Write full wave
    if(fwDetector->fwfYielder != nullptr) {
        fwDetector->fwfYielder->push(FullWaveform(
            fullwave,
            fullwaveIndex,
            min_time,
            max_time,
            beamOrigin,
            beamDir,
            gpstime
        ));
    }
}

bool FullWaveformPulseRunnable::detectPeak(
    int const i,
    int const win_size,
    vector<double> const &fullwave
){
    for (int j = std::max(0, i - 1); j > std::max(0, i - win_size); j--) {
        if (fullwave[j] < eps || fullwave[j] >= fullwave[i]) {
            return false;
        }
    }

    int size = fullwave.size();
    for (int j = std::min(size, i + 1); j < std::min(size, i + win_size); j++){
        if (fullwave[j] < eps || fullwave[j] >= fullwave[i]) {
            return false;
        }
    }

    return true;
}
