#include <ScanningDevice.h>
#include <maths/MathConstants.h>
#include <logging.hpp>
#include <scanner/detector/AbstractDetector.h>
#include <maths/EnergyMaths.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScanningDevice::ScanningDevice(
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
    double const wavelength_m
) :
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
    supportedPulseFreqs_Hz(pulseFreqs)
{
    configureBeam();
    atmosphericExtinction = calcAtmosphericAttenuation();
    cached_Dr2 = receiverDiameter_m * receiverDiameter_m;
}

ScanningDevice::ScanningDevice(ScanningDevice &scdev){
    // TODO Rethink : Implement
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

    if(scdev.scannerHead == nullptr) this->scannerHead = nullptr;
    else this->scannerHead = std::make_shared<ScannerHead>(*scdev.scannerHead);
    if(scdev.beamDeflector == nullptr) this->beamDeflector = nullptr;
    else this->beamDeflector = scdev.beamDeflector->clone();
    if(scdev.detector == nullptr) this->detector = nullptr;
    else this->detector = scdev.detector->clone();
}

// ***  M E T H O D S  *** //
// *********************** //
void ScanningDevice::configureBeam(){
    cached_Bt2 = beamDivergence_rad * beamDivergence_rad;
    beamWaistRadius = (beamQuality * wavelength_m) /
        (M_PI * beamDivergence_rad);
}

// Simulate energy loss from aerial particles (Carlsson et al., 2001)
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
    std::function<void(
        unsigned int, glm::dvec3 &, Rotation &, double const
    )> handlePulseComputation
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
    handlePulseComputation(
        legIndex,
        absoluteBeamOrigin,
        absoluteBeamAttitude,
        currentGpsTime
    );
}

Rotation ScanningDevice::calcAbsoluteBeamAttitude(
    Rotation platformAttitude
){
    Rotation mountRelativeEmitterAttitude =
        scannerHead->getMountRelativeAttitude()
            .applyTo(headRelativeEmitterAttitude);
    return platformAttitude.applyTo(mountRelativeEmitterAttitude)
        .applyTo(beamDeflector->getEmitterRelativeAttitude());
}

void ScanningDevice::computeSubrays(
    std::function<void(
        std::vector<double> const &_tMinMax,
        int const circleStep,
        double const circleStep_rad,
        Rotation &r1,
        double const divergenceAngle,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects
    )> handleSubray,
    std::vector<double> const &tMinMax,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    std::map<double, double> &reflections,
    std::vector<RaySceneIntersection> &intersects
){

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

double ScanningDevice::calcIntensity(
    double const incidenceAngle,
    double const targetRange,
    double const targetReflectivity,
    double const targetSpecularity,
    double const targetSpecularExponent,
    double const targetArea,
    double const radius
) const {
    double const bdrf = targetReflectivity * EnergyMaths::phongBDRF(
        incidenceAngle,
        targetSpecularity,
        targetSpecularExponent
    );
    double const sigma = EnergyMaths::calcCrossSection(
        bdrf, targetArea, incidenceAngle
    );
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
