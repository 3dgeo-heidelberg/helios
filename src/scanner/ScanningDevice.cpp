#include <ScanningDevice.h>
#include <maths/MathConstants.h>
#include <logging.hpp>
#include <scanner/detector/AbstractDetector.h>
#include <maths/EnergyMaths.h>
#if DATA_ANALYTICS >= 2
#include <dataanalytics/HDA_GlobalVars.h>
using namespace helios::analytics;
#endif

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScanningDevice::ScanningDevice(
    size_t const deviceIndex,
    std::string const id,
    double const beamDiv_rad,
    glm::dvec3 const beamOrigin,
    Rotation const beamOrientation,
    std::list<int> const &pulseFreqs,
    double const pulseLength_ns,
    double const averagePower_w,
    double const beamQuality,
    double const efficiency,
    double const receiverDiameter_m,
    double const atmosphericVisibility_km,
    double const wavelength_m,
    std::shared_ptr<UnivarExprTreeNode<double>> rangeErrExpr
) :
    devIdx(deviceIndex),
    id(id),
    headRelativeEmitterPosition(beamOrigin),
    headRelativeEmitterAttitude(beamOrientation),
    beamDivergence_rad(beamDiv_rad),
    pulseLength_ns(pulseLength_ns),
    averagePower_w(averagePower_w),
    beamQuality(beamQuality),
    efficiency(efficiency),
    receiverDiameter_m(receiverDiameter_m),
    visibility_km(atmosphericVisibility_km),
    wavelength_m(wavelength_m),
    supportedPulseFreqs_Hz(pulseFreqs),
    rangeErrExpr(rangeErrExpr)
{
    configureBeam();
    atmosphericExtinction = calcAtmosphericAttenuation();
    cached_Dr2 = receiverDiameter_m * receiverDiameter_m;
}

ScanningDevice::ScanningDevice(ScanningDevice const &scdev){
    this->devIdx = scdev.devIdx;
    this->id = scdev.id;
    this->headRelativeEmitterPosition = scdev.headRelativeEmitterPosition;
    this->headRelativeEmitterAttitude = scdev.headRelativeEmitterAttitude;
    this->beamDivergence_rad = scdev.beamDivergence_rad;
    this->pulseLength_ns = scdev.pulseLength_ns;
    this->averagePower_w = scdev.averagePower_w;
    this->beamQuality = scdev.beamQuality;
    this->efficiency = scdev.efficiency;
    this->receiverDiameter_m = scdev.receiverDiameter_m;
    this->visibility_km = scdev.visibility_km;
    this->wavelength_m = scdev.wavelength_m;
    this->atmosphericExtinction = scdev.atmosphericExtinction;
    this->beamWaistRadius = scdev.beamWaistRadius;
    this->FWF_settings = scdev.FWF_settings;
    this->numRays = scdev.numRays;
    this->supportedPulseFreqs_Hz = scdev.supportedPulseFreqs_Hz;
    this->maxNOR = scdev.maxNOR;
    this->numTimeBins = scdev.numTimeBins;
    this->peakIntensityIndex = scdev.peakIntensityIndex;
    this->time_wave = scdev.time_wave;
    this->rangeErrExpr = scdev.rangeErrExpr;
    this->state_currentPulseNumber = scdev.state_currentPulseNumber;
    this->state_lastPulseWasHit = scdev.state_lastPulseWasHit;
    this->cached_Dr2 = scdev.cached_Dr2;
    this->cached_Bt2 = scdev.cached_Bt2;

    if(scdev.scannerHead == nullptr) this->scannerHead = nullptr;
    else this->scannerHead = std::make_shared<ScannerHead>(*scdev.scannerHead);
    if(scdev.beamDeflector == nullptr) this->beamDeflector = nullptr;
    else this->beamDeflector = scdev.beamDeflector->clone();
    if(scdev.detector == nullptr) this->detector = nullptr;
    else this->detector = scdev.detector->clone();
}

// ***  M E T H O D S  *** //
// *********************** //
void ScanningDevice::prepareSimulation(){
    int const beamSampleQuality = FWF_settings.beamSampleQuality;
    double const radiusStep_rad = beamDivergence_rad/beamSampleQuality;

    // Outer loop over radius steps from beam center to outer edge
    for (int radiusStep = 0; radiusStep < beamSampleQuality; radiusStep++){
        double const subrayDivergenceAngle_rad = radiusStep * radiusStep_rad;

        // Rotate subbeam into divergence step (towards outer rim of the beam cone):
        Rotation r1 = Rotation(Directions::right, subrayDivergenceAngle_rad);

        // Calculate circle step width:
        int circleSteps = (int)(PI_2 * radiusStep);

        // Make sure that central ray is not skipped:
        if (circleSteps == 0) {
            circleSteps = 1;
        }

        double const circleStep_rad = PI_2 / circleSteps;

        // # Loop over sub-rays along the circle
        for (int circleStep = 0; circleStep < circleSteps; circleStep++){
            // Rotate around the circle
            Rotation r2 = Rotation(
                Directions::forward, circleStep_rad * circleStep
            );
            r2 = r2.applyTo(r1);
            // Cache subray generation data
            cached_subrayRotation.push_back(r2);
            cached_subrayDivergenceAngle_rad.push_back(
                subrayDivergenceAngle_rad
            );
        }
    }
}

void ScanningDevice::configureBeam(){
    cached_Bt2 = beamDivergence_rad * beamDivergence_rad;
    beamWaistRadius = (beamQuality * wavelength_m) /
        (M_PI * beamDivergence_rad);
}

// Simulate energy loss from aerial particles (Carlsson et al., 2001)
// Three-dimensional laser radar modelling (Ove Steinvall, Tomas Carlsson) ?
double ScanningDevice::calcAtmosphericAttenuation() const {
    double q;
    double const lambda = wavelength_m * 1e9;
    double const Vm = visibility_km;

    if (lambda < 500 && lambda > 2000) {
        // Do nothing if wavelength is outside range, approximation will be bad
        return 0;
    }

    if (Vm > 50) q = 1.6;
    else if (Vm > 6 && Vm < 50) q = 1.3;
    else q = 0.585 * pow(Vm, 0.33);

    return (3.91 / Vm) * pow((lambda / 0.55), -q);
}

void ScanningDevice::calcRaysNumber() {
    // Count circle steps
    int count = 1;
    for (
        int radiusStep = 0;
        radiusStep < FWF_settings.beamSampleQuality;
        radiusStep++
    ) {
        int circleSteps = (int)(2 * M_PI) * radiusStep;
        count += circleSteps;
    }

    // Update number of rays
    numRays = count;
    std::stringstream ss;
    ss << "Number of subsampling rays ("<< id <<"): " << numRays;
    logging::INFO(ss.str());
}

void ScanningDevice::doSimStep(
    unsigned int legIndex,
    double const currentGpsTime,
    int const simFreq_Hz,
    bool const isActive,
    glm::dvec3 const &platformPosition,
    Rotation const &platformAttitude,
    std::function<void(glm::dvec3 &, Rotation &)> handleSimStepNoise,
    std::function<void(SimulatedPulse const &sp)> handlePulseComputation
){
    // Do what must be done whether active or not
    // ------------------------------------------//
    // Update head attitude (we do this even when the scanner is inactive):
    scannerHead->doSimStep(simFreq_Hz);

    // Stop if not active
    // -------------------//
    if(!isActive) return;

    // Do what active scanner does
    // ----------------------------//
    // Update beam deflector attitude:
    beamDeflector->doSimStep();
    // Check last pulse
    if (!beamDeflector->lastPulseLeftDevice()) return;
    // Pulse counter
    ++state_currentPulseNumber;
    // Calculate absolute beam originWaypoint:
    glm::dvec3 absoluteBeamOrigin = platformPosition +
                                    headRelativeEmitterPosition;
    // Calculate absolute beam attitude:
    Rotation absoluteBeamAttitude = calcAbsoluteBeamAttitude(
        platformAttitude
    );
    // Handle noise
    handleSimStepNoise(absoluteBeamOrigin, absoluteBeamAttitude);
    // Handle pulse computation
    if(hasMechanicalError()) { // Simulated pulse with mechanical error
        Rotation exactAbsoluteBeamAttitude = calcExactAbsoluteBeamAttitude(
            platformAttitude
        );
        double const mechanicalRangeError =
            hasMechanicalRangeErrorExpression() ?
                evalRangeErrorExpression() :
                0.0;
        handlePulseComputation(SimulatedPulse(
            absoluteBeamOrigin,
            absoluteBeamAttitude,
            exactAbsoluteBeamAttitude,
            mechanicalRangeError,
            currentGpsTime,
            legIndex,
            state_currentPulseNumber,
            devIdx
        ));
    }
    else{ // Simulated pulse with NO mechanical error
        handlePulseComputation(SimulatedPulse(
            absoluteBeamOrigin,
            absoluteBeamAttitude,
            currentGpsTime,
            legIndex,
            state_currentPulseNumber,
            devIdx
        ));
    }
}

Rotation ScanningDevice::calcAbsoluteBeamAttitude(
    Rotation const &platformAttitude
){
    Rotation mountRelativeEmitterAttitude =
        scannerHead->getMountRelativeAttitude()
            .applyTo(headRelativeEmitterAttitude);
    return platformAttitude.applyTo(mountRelativeEmitterAttitude)
        .applyTo(beamDeflector->getEmitterRelativeAttitude());
}

Rotation ScanningDevice::calcExactAbsoluteBeamAttitude(
    Rotation const &platformAttitude
){
    Rotation exactMountRelativeEmitterAttitude =
        scannerHead->getExactMountRelativeAttitude()
            .applyTo(headRelativeEmitterAttitude);
    return platformAttitude.applyTo(exactMountRelativeEmitterAttitude)
        .applyTo(beamDeflector->getExactEmitterRelativeAttitude());
}

void ScanningDevice::computeSubrays(
    std::function<void(
        Rotation const &subrayRotation,
        double const divergenceAngle,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects
#if DATA_ANALYTICS >=2
       ,bool &subrayHit,
        std::vector<double> &subraySimRecord
#endif
    )> handleSubray,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    std::map<double, double> &reflections,
    std::vector<RaySceneIntersection> &intersects
#if DATA_ANALYTICS >=2
   ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
){
    size_t const numSubrays = cached_subrayRotation.size();
    for(size_t i = 0 ; i < numSubrays ; ++i) {
    #if DATA_ANALYTICS >=2
        bool subrayHit;
    #endif
#if DATA_ANALYTICS >=2
        std::vector<double> subraySimRecord(
            14, std::numeric_limits<double>::quiet_NaN()
        );
#endif
        handleSubray(
            cached_subrayRotation[i],
            cached_subrayDivergenceAngle_rad[i],
            intersectionHandlingNoiseSource,
            reflections,
            intersects
#if DATA_ANALYTICS >=2
           ,subrayHit,
            subraySimRecord
#endif
        );
#if DATA_ANALYTICS >=2
        HDA_GV.incrementGeneratedSubraysCount();
        subraySimRecord[0] = (double) subrayHit;
        subraySimRecord[1] = cached_subrayDivergenceAngle_rad[i];
        pulseRecorder->recordSubraySimulation(subraySimRecord);
#endif
    }
}

bool ScanningDevice::initializeFullWaveform(
    double const minHitDist_m,
    double const maxHitDist_m,
    double &minHitTime_ns,
    double &maxHitTime_ns,
    double &nsPerBin,
    double &distanceThreshold,
    int &peakIntensityIndex,
    int &numFullwaveBins
){
    // Calc time at minimum and maximum distance
    // (i.e. total beam time in fwf signal)
    nsPerBin = FWF_settings.binSize_ns;
    peakIntensityIndex = this->peakIntensityIndex;
    double const peakFactor = peakIntensityIndex * nsPerBin;
    // Time until first maximum minus rising flank
    minHitTime_ns = minHitDist_m / SPEEDofLIGHT_mPerNanosec - peakFactor;
    // Time until last maximum time for signal decay with 1 bin for buffer
    maxHitTime_ns = maxHitDist_m / SPEEDofLIGHT_mPerNanosec +
                    pulseLength_ns - peakFactor +
                    nsPerBin; // 1 bin for buffer

    // Calc ranges and threshold
    double hitTimeDelta_ns = maxHitTime_ns - minHitTime_ns;
    double const maxFullwaveRange_ns = FWF_settings.maxFullwaveRange_ns;
    distanceThreshold = maxHitDist_m;
    if(maxFullwaveRange_ns > 0.0 && hitTimeDelta_ns > maxFullwaveRange_ns){
        hitTimeDelta_ns = maxFullwaveRange_ns;
        maxHitTime_ns = minHitTime_ns + maxFullwaveRange_ns;
        distanceThreshold = SPEEDofLIGHT_mPerNanosec * maxFullwaveRange_ns;
    }

    // Check if full wave is possible
    if(
        (detector->cfg_device_rangeMin_m / SPEEDofLIGHT_mPerNanosec)
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

double ScanningDevice::calcIntensity(
    double const incidenceAngle,
    double const targetRange,
    Material const &mat,
    double const targetArea,
    double const radius
#if DATA_ANALYTICS >=2
   ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
) const {
    double bdrf = 0, sigma = 0;
    if(mat.isPhong()) {
        bdrf = mat.reflectance * EnergyMaths::phongBDRF(
            incidenceAngle,
            mat.specularity,
            mat.specularExponent
        );
        sigma = EnergyMaths::calcCrossSection(
            bdrf, targetArea, incidenceAngle
        );
    }
    else if(mat.isLambert()){
        bdrf = mat.reflectance;
        sigma = EnergyMaths::calcCrossSection(
            bdrf, targetArea, incidenceAngle
        );
    }
    else if(mat.isDirectionIndependent()){
        bdrf = mat.reflectance/std::cos(incidenceAngle);  // Alt. 1/cos(incid)
        sigma = EnergyMaths::calcCrossSection(
            bdrf, targetArea, incidenceAngle
        );
        if(sigma < 0) sigma = -sigma;
    }
    else{
        std::stringstream ss;
        ss  << "Unexpected lighting model for material \""
            << mat.name << "\"";
        logging::ERR(ss.str());
    }
    double const receivedPower = EnergyMaths::calcReceivedPower(
        averagePower_w,
        wavelength_m,
        targetRange,
        detector->cfg_device_rangeMin_m,
        radius,
        beamWaistRadius,
        cached_Dr2,
        cached_Bt2,
        efficiency,
        atmosphericExtinction,
        sigma
    ) * 1000000000.0;
#if DATA_ANALYTICS >= 2
    std::vector<double> calcIntensityRecord(
        11, std::numeric_limits<double>::quiet_NaN()
    );
    calcIntensityRecord[3] = incidenceAngle;
    calcIntensityRecord[4] = targetRange;
    calcIntensityRecord[5] = targetArea;
    calcIntensityRecord[6] = radius;
    calcIntensityRecord[7] = bdrf;
    calcIntensityRecord[8] = sigma;
    calcIntensityRecord[9] = receivedPower;
    calcIntensityRecord[10] = 0; // By default, assume the point isn't captured
    calcIntensityRecords.push_back(calcIntensityRecord);
#endif
    return receivedPower;
}
double ScanningDevice::calcIntensity(
    double const targetRange,
    double const radius,
    double const sigma
) const {
    return EnergyMaths::calcReceivedPower(
        averagePower_w,
        wavelength_m,
        targetRange,
        detector->cfg_device_rangeMin_m,
        radius,
        beamWaistRadius,
        cached_Dr2,
        cached_Bt2,
        efficiency,
        atmosphericExtinction,
        sigma
    ) * 1000000000.0;
}


// ***  GETTERs and SETTERs  *** //
// ***************************** //
void ScanningDevice::setLastPulseWasHit(bool const value){
    if (value == state_lastPulseWasHit) return;
    //TODO see https://www.codeproject.com/Articles/12362/A-quot-synchronized-quot-statement-for-C-like-in-J
    state_lastPulseWasHit = value;
}