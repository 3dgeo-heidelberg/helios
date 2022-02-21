// TODO Pending : This implementation is calling scanner setLastPulseWasHit
// Is this thread safe?
#include "FullWaveformPulseRunnable.h"

#include "logging.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "maths/Rotation.h"
#include "MarquardtFitter.h"
#include <TimeWatcher.h>
#include <maths/RayUtils.h>

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
){
	// Compute beam direction
	glm::dvec3 beamDir = absoluteBeamAttitude.applyTo(Directions::forward);

	// NOTE:
	// With beam divergence / full waveform being simulated, this is not perfect, since a sub-ray
	// might hit the scene even if the central ray does now. However, this check is a very important
	// performance optimization, so we should keep it nevertheless. sbecht 2016-04-24

	// Early abort if central axis of the beam does not intersect with the scene:
	vector<double> tMinMax = scene.getAABB()->getRayIntersection(
	    absoluteBeamOrigin,
	    beamDir
    );
	if (tMinMax.empty()) {
		logging::DEBUG("Early abort - beam does not intersect with the scene");
		detector->scanner->setLastPulseWasHit(false);
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
void FullWaveformPulseRunnable::computeSubrays(
    vector<double> const &tMinMax,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    std::map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
){
    double radiusStep_rad = detector->scanner->getBeamDivergence()/
        detector->scanner->FWF_settings.beamSampleQuality;

    // Outer loop over radius steps from beam center to outer edge
    int beamSampleQuality = detector->scanner->FWF_settings.beamSampleQuality;
    for (int radiusStep = 0; radiusStep < beamSampleQuality; radiusStep++){
        double subrayDivergenceAngle_rad = radiusStep * radiusStep_rad;

        // Rotate subbeam into divergence step (towards outer rim of the beam cone):
        Rotation r1 = Rotation(Directions::right, subrayDivergenceAngle_rad);

        // Calculate circle step width:
        int circleSteps = (int)(2 * M_PI) * radiusStep;

        // Make sure that central ray is not skipped:
        if (circleSteps == 0) {
            circleSteps = 1;
        }

        double circleStep_rad = (2 * M_PI) / circleSteps;

        // # Loop over sub-rays along the circle
        for (int circleStep = 0; circleStep < circleSteps; circleStep++){
            handleSubray(
                tMinMax,
                circleStep,
                circleStep_rad,
                r1,
                subrayDivergenceAngle_rad,
                intersectionHandlingNoiseSource,
                reflections,
                intersects
            );
        }
    }
}

void FullWaveformPulseRunnable::handleSubray(
    vector<double> const &tMinMax,
    int circleStep,
    double circleStep_rad,
    Rotation &r1,
    double divergenceAngle,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
){
    // Rotate around the circle:
    Rotation r2 = Rotation(Directions::forward, circleStep_rad * circleStep);
    r2 = r2.applyTo(r1);

    glm::dvec3 subrayDirection = absoluteBeamAttitude
        .applyTo(r2)
        .applyTo(Directions::forward);

    glm::dvec3 subrayOrigin(absoluteBeamOrigin);
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
            if(!detector->scanner->isFixedIncidenceAngle()) {
                incidenceAngle =
                    intersect->prim->getIncidenceAngle_rad(
                        absoluteBeamOrigin,
                        subrayDirection,
                        intersect->point
                    );
            }

            // Distance between beam origin and intersection:
            double distance = glm::distance(
                intersect->point,
                absoluteBeamOrigin
            );

            if(
                detector->cfg_device_rangeMin_m > distance ||
                detector->cfg_device_rangeMax_m < distance
            ) continue;

            // Distance between beam's center line and intersection point:
            double radius = sin(divergenceAngle) * distance;
            double const targetArea =
                detector->scanner->calcFootprintArea(distance) /
                (double) detector->scanner->getNumRays();
            double intensity = 0.0;
            if(intersect->prim->canComputeSigmaWithLadLut()){
                // LadLut based intensity computation
                double sigma = intersect->prim->computeSigmaWithLadLut(
                    subrayDirection
                );
                intensity = calcIntensity(distance, radius, sigma);
            }
            else{
                // Standard intensity computation
                intensity = calcIntensity(
                    incidenceAngle,
                    distance,
                    intersect->prim->material->reflectance,
                    intersect->prim->material->specularity,
                    intersect->prim->material->specularExponent,
                    targetArea,
                    radius
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
                    // Move subray originWaypoint outside primitive
                    subrayOrigin = outsideIntersectionPoint +
                                   0.00001 * subrayDirection;
                    rayContinues = true;
                }
                else{ // Update distance considering noise
                    distance = glm::distance(
                        ihr.getIntersectionPoint(),
                        absoluteBeamOrigin
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
        detector->scanner->setLastPulseWasHit(false);
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
        beamDir,
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

        double entryDistance = it->first;

        if (entryDistance > maxHitDist_m) {
            maxHitDist_m = entryDistance;
        }
        if (entryDistance < minHitDist_m) {
            minHitDist_m = entryDistance;
        }
    }
}

bool FullWaveformPulseRunnable::initializeFullWaveform(
    double minHitDist_m,
    double maxHitDist_m,
    double &minHitTime_ns,
    double &maxHitTime_ns,
    double &nsPerBin,
    double &distanceThreshold,
    int &peakIntensityIndex,
    int &numFullwaveBins
){
    // Calc time at minimum and maximum distance
    // (i.e. total beam time in fwf signal)
    peakIntensityIndex = detector->scanner->peakIntensityIndex;
    nsPerBin = detector->scanner->FWF_settings.binSize_ns;
    double const peakFactor = peakIntensityIndex * nsPerBin;
    // Time until first maximum minus rising flank
    minHitTime_ns = minHitDist_m / cfg_speedOfLight_mPerNanosec - peakFactor;
    // Time until last maximum time for signal decay with 1 bin for buffer
    maxHitTime_ns = maxHitDist_m / cfg_speedOfLight_mPerNanosec +
        detector->scanner->getPulseLength_ns() - peakFactor +
        nsPerBin; // 1 bin for buffer

    // Calc ranges and threshold
    double hitTimeDelta_ns = maxHitTime_ns - minHitTime_ns;
    double maxFullwaveRange_ns =
        detector->scanner->FWF_settings.maxFullwaveRange_ns;
    distanceThreshold = maxHitDist_m;
    if(maxFullwaveRange_ns > 0.0 && hitTimeDelta_ns > maxFullwaveRange_ns){
        hitTimeDelta_ns = maxFullwaveRange_ns;
        maxHitTime_ns = minHitTime_ns + maxFullwaveRange_ns;
        distanceThreshold = cfg_speedOfLight_mPerNanosec * maxFullwaveRange_ns;
    }

    // Check if full wave is possible
    if(
        (detector->cfg_device_rangeMin_m / cfg_speedOfLight_mPerNanosec)
        > minHitTime_ns
    ) {
        return false;
    }

    // Compute fullwave variables
    numFullwaveBins = ((int)std::ceil(maxHitTime_ns/nsPerBin)) -
        ((int)ceil(minHitTime_ns/nsPerBin));

    // update maxHitTime to fit the discretized fullwave bins
    // minus 1 is necessary as the minimum is in bin #0
    maxHitTime_ns = minHitTime_ns + (numFullwaveBins - 1) * nsPerBin;

    return true;
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
    vector<double> const &time_wave = detector->scanner->time_wave;
    map<double, double>::const_iterator it;
    for (it = reflections.begin(); it != reflections.end(); ++it) {
        double const entryDistance_m = it->first;
        if(entryDistance_m > distanceThreshold) continue;
        double const entryIntensity = it->second;
        double const wavePeakTime_ns = entryDistance_m /
            cfg_speedOfLight_mPerNanosec; // in nanoseconds
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
    numReturns = 0;
    int win_size = (int)(detector->scanner->FWF_settings.winSize_ns/nsPerBin);
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
        if (calcEchowidth) {
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
        double distance = cfg_speedOfLight_mPerNanosec *
            (i * nsPerBin + minHitTime_ns);

        // Build list of objects that produced this return
        double minDifference = numeric_limits<double>::max();
        shared_ptr<RaySceneIntersection> closestIntersection = nullptr;

        for (RaySceneIntersection intersect : intersects) {
            double intersectDist = glm::distance(
                intersect.point, absoluteBeamOrigin);

            if (std::fabs(intersectDist - distance) < minDifference) {
                minDifference = std::fabs(intersectDist - distance);
                closestIntersection = make_shared<RaySceneIntersection>(
                    intersect);
            }
        }

        string hitObject;
        int classification = 0;
        if (closestIntersection != nullptr) {
            hitObject = closestIntersection->prim->part->mId;
            classification =
                closestIntersection->prim->material->classification;
        }

        // Build measurement
        Measurement tmp;
        tmp.beamOrigin = absoluteBeamOrigin;
        tmp.beamDirection = beamDir;
        tmp.distance = distance;
        tmp.echo_width = echo_width;
        tmp.intensity = fullwave.at(i);
        tmp.fullwaveIndex = currentPulseNum;
        tmp.hitObjectId = hitObject;
        tmp.returnNumber = numReturns + 1;
        tmp.classification = classification;
        tmp.gpsTime = currentGpsTime;

        pointsMeasurement.push_back(tmp);
        ++numReturns;

        // Check if maximum number of returns per pulse has been reached
        if(!fwDetector->scanner->checkMaxNOR(numReturns)) break;
    }

}

void FullWaveformPulseRunnable::exportOutput(
    std::vector<double> &fullwave,
    int &numReturns,
    std::vector<Measurement> &pointsMeasurement,
    glm::dvec3 &beamDir,
    double minHitTime_ns,
    double maxHitTime_ns,
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
                allMeasurements,
                allMeasurementsMutex,
                cycleMeasurements,
                cycleMeasurementsMutex
            );
        }
        if(writeWaveform) {
            detector->scanner->setLastPulseWasHit(true);
            captureFullWave(
                fullwave,
                currentPulseNum,
                minHitTime_ns,
                maxHitTime_ns,
                absoluteBeamOrigin,
                beamDir,
                currentGpsTime,
                detector->scanner->isFullWaveNoise(),
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

// Space distribution equation to calculate the beam energy decreasing the further away from the center (Carlsson et al., 2001)
double FullWaveformPulseRunnable::calcEmmitedPower(double radius, double targetRange) {
    double const I0 = detector->scanner->getAveragePower();
    double const lambda = detector->scanner->getWavelength();
    double const R = targetRange;
    double const R0 = detector->cfg_device_rangeMin_m;
    double const r = radius;
    double const w0 = detector->scanner->getBeamWaistRadius();
    double const denom = M_PI * w0 * w0;
    double const omega = (lambda * R) / denom;
    double const omega0 = (lambda * R0) / denom;
    double const w = w0 * sqrt(omega0 * omega0 + omega * omega);

    return I0 * exp((-2 * r * r) / (w * w));
}

// Calculate the strength of the laser going back to the detector
double FullWaveformPulseRunnable::calcIntensity(
    double incidenceAngle,
    double targetRange,
    double targetReflectivity,
    double targetSpecularity,
    double targetSpecularExponent,
    double targetArea,
    double radius
){
    double emmitedPower = calcEmmitedPower(radius, targetRange);
    double intensity = AbstractPulseRunnable::calcReceivedPower(
        emmitedPower,
        targetRange,
        incidenceAngle,
        targetReflectivity,
        targetSpecularity,
        targetSpecularExponent,
        targetArea
    );
    return intensity * 1000000000.0;
}

double FullWaveformPulseRunnable::calcIntensity(
    double targetRange,
    double radius,
    double sigma
){
    double emmitedPower = calcEmmitedPower(radius, targetRange);
    double intensity = AbstractPulseRunnable::calcReceivedPower(
        emmitedPower,
        targetRange,
        sigma
    );
    return intensity * 1000000000.0;
}


void FullWaveformPulseRunnable::captureFullWave(
    vector<double> & fullwave,
    int fullwaveIndex,
    double min_time,
    double max_time,
    glm::dvec3 & beamOrigin,
    glm::dvec3 & beamDir,
    double gpstime,
    bool fullWaveNoise,
    RandomnessGenerator<double> &rg2
){
    // Add noise to the fullwave
    if(fullWaveNoise) {
        for (double &fw : fullwave) {
            fw += rg2.normalDistributionNext() *
                  this->detector->cfg_device_accuracy_m / 2.0;
        }
    }

    // Write full wave
    fwDetector->writeFullWave(
        fullwave,
        fullwaveIndex,
        min_time,
        max_time,
        beamOrigin,
        beamDir,
        gpstime
    );
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
