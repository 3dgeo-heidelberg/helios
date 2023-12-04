// TODO Pending : This implementation is calling scanner setLastPulseWasHit
// Is this thread safe?
#include "FullWaveformPulseRunnable.h"

#include "logging.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "maths/Rotation.h"
#include <maths/EnergyMaths.h>
#include <TimeWatcher.h>
#include <maths/RayUtils.h>
#include <filems/facade/FMSFacade.h>
#include <scanner/detector/FullWaveform.h>
#include <scanner/Scanner.h>

#if DATA_ANALYTICS >= 2
#include <dataanalytics/HDA_GlobalVars.h>
using helios::analytics::HDA_GV;

#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#endif

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
#if DATA_ANALYTICS >= 2
   ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    // Deferred/lazy initialization
    initialize();
	// Compute beam direction
	glm::dvec3 beamDir = pulse.computeDirection();
    // Capture pulse if requested
    if(scanner->isWritePulse()) capturePulse(beamDir);

	// NOTE:
	// With beam divergence / full waveform being simulated, this is not perfect, since a sub-ray
	// might hit the scene even if the central ray does now. However, this check is a very important
	// performance optimization, so we should keep it nevertheless. sbecht 2016-04-24

	// Early abort if central axis of the beam does not intersect with the scene:
	vector<double> const tMinMax = scene.getAABB()->getRayIntersection(
	    pulse.getOriginRef(),
	    beamDir
    );
#if DATA_ANALYTICS >= 2
    HDA_GV.incrementGeneratedRaysBeforeEarlyAbortCount();
#endif
	if (checkEarlyAbort(tMinMax)) {
		logging::DEBUG("Early abort - beam does not intersect with the scene");
		scanner->setLastPulseWasHit(false, pulse.getDeviceIndex());
		return;
	}
#if DATA_ANALYTICS >= 2
    HDA_GV.incrementGeneratedRaysAfterEarlyAbortCount();
#endif

	// Ray casting (find intersections)
	map<double, double> reflections;
	vector<RaySceneIntersection> intersects;
#if DATA_ANALYTICS >= 2
    std::vector<std::vector<double>> calcIntensityRecords;
    std::vector<std::vector<int>> calcIntensityIndices;
#endif
	computeSubrays(
	    intersectionHandlingNoiseSource,
	    reflections,
	    intersects
#if DATA_ANALYTICS >= 2
      ,calcIntensityRecords,
       pulseRecorder
#endif
    );

#if DATA_ANALYTICS >= 2
    for(std::vector<double> &calcIntensityRecord : calcIntensityRecords) {
        calcIntensityIndices.push_back(
            std::vector<int>({pulse.getPulseNumber()})
        );
    }
#endif

	// Digest intersections
	digestIntersections(
	    apMatrix,
	    randGen,
	    randGen2,
	    beamDir,
	    reflections,
	    intersects
#if DATA_ANALYTICS >= 2
       ,calcIntensityRecords,
        calcIntensityIndices,
        pulseRecorder
#endif
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
    NoiseSource<double> &intersectionHandlingNoiseSource,
    std::map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
#if DATA_ANALYTICS >= 2
   ,std::vector<std::vector<double>> &calcIntensityRecords,
    std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    scanner->computeSubrays(
        [&] (
            Rotation const &subrayRotation,
            double const divergenceAngle,
            int const subrayRadiusStep,
            NoiseSource<double> &intersectionHandlingNoiseSource,
            std::map<double, double> &reflections,
            vector<RaySceneIntersection> &intersects
#if DATA_ANALYTICS >= 2
           ,bool &subrayHit,
            std::vector<double> &subraySimRecord
#endif
        ) -> void {
            handleSubray(
                subrayRotation,
                divergenceAngle,
                subrayRadiusStep,
                intersectionHandlingNoiseSource,
                reflections,
                intersects
#if DATA_ANALYTICS >= 2
               ,subrayHit,
                subraySimRecord,
                calcIntensityRecords
#endif
            );
        },
        intersectionHandlingNoiseSource,
        reflections,
        intersects,
        pulse.getDeviceIndex()
#if DATA_ANALYTICS >= 2
       ,pulseRecorder
#endif
    );
}

void FullWaveformPulseRunnable::handleSubray(
    Rotation const &subrayRotation,
    double const divergenceAngle,
    int const subrayRadiusStep,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
#if DATA_ANALYTICS >= 2
   ,bool &subrayHit,
    std::vector<double> &subraySimRecord,
    std::vector<std::vector<double>> &calcIntensityRecords
#endif
){
#if DATA_ANALYTICS >= 2
    subrayHit = false;
#endif
    // Rotate around the circle:
    glm::dvec3 subrayDirection = pulse.getAttitude().applyTo(subrayRotation)
        .applyTo(Directions::forward);
    vector<double> tMinMax = scene.getAABB()->getRayIntersection(
        pulse.getOriginRef(),
        subrayDirection
    );
#if DATA_ANALYTICS >= 2
    // Subray simulation record
    glm::dvec3 rayDirection = pulse.computeDirection();
    subraySimRecord[2] = glm::l2Norm(rayDirection); // Ray norm
    subraySimRecord[3] = glm::l2Norm(subrayDirection); // Subray norm
    subraySimRecord[4] = glm::angle( // Angle between ray and subray
        rayDirection,
        subrayDirection
    );
    subraySimRecord[5] = (rayDirection[0] < 0) == (subrayDirection[0] < 0);
    subraySimRecord[6] = (tMinMax.size()<1) ? 0 : tMinMax[0];
    subraySimRecord[7] = (tMinMax.size()<1) ? 0 : tMinMax[1];
    subraySimRecord[8] = subrayDirection.x;
    subraySimRecord[9] = subrayDirection.y;
    subraySimRecord[10] = subrayDirection.z;
    subraySimRecord[11] = rayDirection.x;
    subraySimRecord[12] = rayDirection.y;
    subraySimRecord[13] = rayDirection.z;
#endif
    if(checkEarlyAbort(tMinMax)) return;

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
#if DATA_ANALYTICS >= 2
            HDA_GV.incrementSubrayIntersectionCount();
            subrayHit = true;
#endif
            // Incidence angle:
            if (!scanner->isFixedIncidenceAngle()) {
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
            if (detector->isDistanceNotInRange(distance)) continue;

            // Distance between beam's center line and intersection point:
            double const radius = sin(divergenceAngle) * distance;
            double intensity = 0.0;
            if (intersect->prim->canComputeSigmaWithLadLut()) {
                // LadLut based intensity computation
                double const sigma = intersect->prim->computeSigmaWithLadLut(
                    subrayDirection
                );
                // TODO Rethink : Make an energy model for LadLuts too
                intensity = scanner->calcIntensity(
                    distance, radius, sigma, pulse.getDeviceIndex()
                );
            } else {
                // Lighting-based intensity computation
#if DATA_ANALYTICS >= 2
                HDA_GV.incrementIntensityComputationsCount();
#endif
                intensity = scanner->calcIntensity(
                    incidenceAngle,
                    distance,
                    *intersect->prim->material,
                    radius,
                    subrayRadiusStep,
                    pulse.getDeviceIndex()
#if DATA_ANALYTICS >= 2
                   ,calcIntensityRecords
#endif
                );
            }


            // Intersection handling
            if (intersect->prim->canHandleIntersections()) {
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
                } else { // Update distance considering noise
                    distance = glm::distance(
                        ihr.getIntersectionPoint(),
                        pulse.getOriginRef()
                    );
                }
            }
            if (!rayContinues) { // If ray is not continuing
                // Then register hit by default
                reflections.insert(
                    pair<double, double>(distance, intensity)
                );
                intersects.push_back(*intersect);
            }
#if DATA_ANALYTICS >= 2
            // TODO Rethink : At this point, double insertion has happened
            std::vector<double> &calcIntensityRecord =
                calcIntensityRecords.back();
            calcIntensityRecord[0] = intersect->point.x;
            calcIntensityRecord[1] = intersect->point.y;
            calcIntensityRecord[2] = intersect->point.z;
        }
        else {
            HDA_GV.incrementSubrayNonIntersectionCount();
#endif
        }
    }
#if DATA_ANALYTICS >= 2
    if(subrayHit) HDA_GV.incrementIntersectiveSubraysCount();
    else HDA_GV.incrementNonIntersectiveSubraysCount();
#endif
}

void FullWaveformPulseRunnable::digestIntersections(
    std::vector<std::vector<double>> &apMatrix,
    RandomnessGenerator<double> &randGen,
    RandomnessGenerator<double> &randGen2,
    glm::dvec3 &beamDir,
    std::map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects
#if DATA_ANALYTICS >= 2
   ,vector<vector<double>> &calcIntensityRecords,
    vector<vector<int>> &calcIntensityIndices,
    std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    // Find maximum hit distances
    double minHitDist_m, maxHitDist_m;
    findMaxMinHitDistances(reflections, minHitDist_m, maxHitDist_m);

    // If nothing was hit, get out of here
    if (maxHitDist_m < 0) {
        scanner->setLastPulseWasHit(false, pulse.getDeviceIndex());
#if DATA_ANALYTICS >= 2
        if(!calcIntensityRecords.empty()) {
            pulseRecorder->recordIntensityCalculation(
                calcIntensityRecords,
                calcIntensityIndices
            );
        }
#endif
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
    )){
#if DATA_ANALYTICS >= 2
        pulseRecorder->recordIntensityCalculation(
            calcIntensityRecords,
            calcIntensityIndices
        );
#endif
        return;
    }
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
#if DATA_ANALYTICS >= 2
       ,calcIntensityRecords,
        calcIntensityIndices,
        pulseRecorder
#endif
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
#if DATA_ANALYTICS >= 2
       ,calcIntensityRecords,
        calcIntensityIndices,
        pulseRecorder
#endif
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
#if DATA_ANALYTICS >= 2
   ,std::vector<std::vector<double>> &calcIntensityRecords,
    std::vector<std::vector<int>> &calcIntensityIndices,
    std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
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

#if DATA_ANALYTICS >= 2
    std::unordered_set<std::size_t> capturedIndices;
#endif

    for (int i = 0; i < numFullwaveBins; ++i) {
        // Skip iteration if the handling of the bin_i does not accept the hit
        if(handleFullWaveformBin(
            fullwave,
            fit,
            echo_width,
            i,
            win_size,
            nsPerBin
        )) continue;

        // Compute distance
        double distance = SPEEDofLIGHT_mPerNanosec *
            (i * nsPerBin + minHitTime_ns);

        // Build list of objects that produced this return
        double minDifference = numeric_limits<double>::max();
        shared_ptr<RaySceneIntersection> closestIntersection = nullptr;

#ifdef DATA_ANALYTICS
        size_t intersectionIdx = 0;
        size_t closestIntersectionIdx = 0;
#endif
        for (RaySceneIntersection intersect : intersects) {
            double const intersectDist = glm::distance(
                intersect.point, pulse.getOriginRef());
            double const absDistDiff = std::fabs(intersectDist-distance);

            if (absDistDiff < minDifference) {
                minDifference = absDistDiff;
                closestIntersection = make_shared<RaySceneIntersection>(
                    intersect);
#ifdef DATA_ANALYTICS
                closestIntersectionIdx = intersectionIdx;
#endif
            }
#ifdef DATA_ANALYTICS
            ++intersectionIdx;
#endif
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
#if DATA_ANALYTICS >= 2
        capturedIndices.insert(closestIntersectionIdx);
#endif

        // Check if maximum number of returns per pulse has been reached
        if(!scanner->checkMaxNOR(numReturns, devIdx)) break;
    }
#if DATA_ANALYTICS >= 2
    // Record all non captured points and remove them from records
    size_t const numRecords = calcIntensityRecords.size();
    size_t nonCapturedCount = 0;
    for(size_t i = 0 ; i < numRecords ; ++i){
        // TODO Rethink : Why with BSQ==1 there is n=2 instead of n=1 here ?
        if(capturedIndices.find(i) == capturedIndices.end()){
            pulseRecorder->recordIntensityCalculation(
                calcIntensityRecords[i-nonCapturedCount],
                calcIntensityIndices[i-nonCapturedCount]
            );
            calcIntensityRecords.erase(
                calcIntensityRecords.begin()+i-nonCapturedCount
            );
            calcIntensityIndices.erase(
                calcIntensityIndices.begin()+i-nonCapturedCount
            );
            ++nonCapturedCount;
        }
    }
#endif
}

bool FullWaveformPulseRunnable::handleFullWaveformBin(
    std::vector<double> const &fullwave,
    MarquardtFitter &fit,
    double &echoWidth,
    int const binIndex,
    int const winSize,
    double const nsPerBin
){
    if (fullwave[binIndex] < eps) return true;

    // Continue with next iteration if there is no peak
    if(!detectPeak(binIndex, winSize, fullwave)) return true;

    // Gaussian model fitting
    if (scanner->isCalcEchowidth()) {
        try {
            fit.setParameters(
                vector<double>{0, fullwave[binIndex], (double) binIndex, 1}
            );
            fit.fitData();
        }
        catch (std::exception &e) {
            logging::ERR(e.what());
            return true;
        }
        echoWidth = fit.getParameters()[3];
        echoWidth = echoWidth * nsPerBin;

        if (echoWidth < 0.1) { // TODO Pending : 0.1 to threshold variable
            return true;
        }
    }
    // Return false as the bin corresponds to an accepted hit
    return false;
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
#if DATA_ANALYTICS >= 2
   ,std::vector<std::vector<double>> &calcIntensityRecords,
    std::vector<std::vector<int>> &calcIntensityIndices,
    std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    // ############ END Extract points from waveform data ################
    if (numReturns > 0) {
#if DATA_ANALYTICS >= 2
        size_t i = 0;
#endif
        for (Measurement & pm : pointsMeasurement) {
            pm.pulseReturnNumber = numReturns;
            capturePoint(
                pm,
                randGen,
                scanner->allMeasurements.get(),
                scanner->allMeasurementsMutex.get(),
                scanner->cycleMeasurements.get(),
                scanner->cycleMeasurementsMutex.get()
#if DATA_ANALYTICS >= 2
               ,calcIntensityRecords[i],
                calcIntensityIndices[i],
                pulseRecorder
#endif
            );
#if DATA_ANALYTICS >= 2
            ++i;
#endif
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

    int const size = fullwave.size();
    for (int j = std::min(size, i + 1); j < std::min(size, i + win_size); j++){
        if (fullwave[j] < eps || fullwave[j] >= fullwave[i]) {
            return false;
        }
    }

    return true;
}
