#include <scanner/SingleScanner.h>
#include <scanner/detector/AbstractDetector.h>
#include <maths/WaveMaths.h>

#ifdef PYTHON_BINDING
#include "PyDetectorWrapper.h"
#endif

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SingleScanner::SingleScanner(
    double const beamDiv_rad,
    glm::dvec3 const beamOrigin,
    Rotation const beamOrientation,
    std::list<int> const &pulseFreqs,
    double const pulseLength_ns,
    std::string const id,
    double const averagePower,
    double const beamQuality,
    double const efficiency,
    double const receiverDiameter,
    double const atmosphericVisibility,
    int const wavelength,
    bool const writeWaveform,
    bool const calcEchowidth,
    bool const fullWaveNoise,
    bool const platformNoiseDisabled
) :
    Scanner(
        id,
        pulseFreqs,
        writeWaveform,
        calcEchowidth,
        fullWaveNoise,
        platformNoiseDisabled
    ),
    scanDev(
        0,
        id,
        beamDiv_rad,
        beamOrigin,
        beamOrientation,
        pulseFreqs,
        pulseLength_ns,
        averagePower,
        beamQuality,
        efficiency,
        receiverDiameter,
        atmosphericVisibility,
        wavelength  / 1000000000.0
    )
{
    // Report scanner state through logging system
    logging::INFO(toString());
}

SingleScanner::SingleScanner(SingleScanner &scanner) :
    Scanner(scanner),
    scanDev(scanner.scanDev)
{}

// ***   C L O N E   *** //
// ********************* //
std::shared_ptr<Scanner> SingleScanner::clone() {
    std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(*this);
    _clone(*scanner);
    return scanner;
}

void SingleScanner::_clone(Scanner &sc) const{
    // Call parent clone method
    Scanner::_clone(sc);
    // Clone attributes from SingleScanner class itself
    //SingleScanner & ssc = static_cast<SingleScanner &>(sc);  // Not used
    //sc.scanDev = scanDev;  // Not necessary because of copy constructor call
}


// ***  SIM STEP UTILS  *** //
// ************************ //
void SingleScanner::onLegComplete(){
    // Call parent handler for on leg complete events
    Scanner::onLegComplete();
    // Call detector's handle method for on leg complete events
    getDetector(0)->onLegComplete();
}

// ***   M E T H O D S   *** //
// ************************* //
void SingleScanner::applySettings(
    std::shared_ptr<ScannerSettings> settings, size_t const idx
){
    // Configure scanner and scanning device
    setPulseFreq_Hz(settings->pulseFreq_Hz);
    setActive(settings->active);
    setBeamDivergence(settings->beamDivAngle, 0);
    trajectoryTimeInterval_ns = settings->trajectoryTimeInterval*1000000000.0;
    scanDev.configureBeam();

    // Configure other components
    getDetector(0)->applySettings(settings);
    getScannerHead(0)->applySettings(settings);
    getBeamDeflector(0)->applySettings(settings);
}

void SingleScanner::doSimStep(
    unsigned int legIndex, double const currentGpsTime
){
    // Check whether the scanner is active or not
    bool const _isActive = isActive();

    // Simulate scanning devices
    scanDev.doSimStep(
        legIndex,
        currentGpsTime,
        cfg_setting_pulseFreq_Hz,
        _isActive,
        platform->getAbsoluteMountPosition(),
        platform->getAbsoluteMountAttitude(),
        [&] (glm::dvec3 &origin, Rotation &attitude) -> void {
            handleSimStepNoise(origin, attitude);
        },
        [&] (SimulatedPulse const &sp) -> void{
            spp->handlePulseComputation(sp);
        }
    );

    // If the scanner is inactive, stop here:
    if (!_isActive) return;

    // Handle trajectory output
    handleTrajectoryOutput(currentGpsTime);
}

void SingleScanner::prepareDiscretization(size_t const idx){
    setNumTimeBins(
        getPulseLength_ns(0) / getFWFSettings(0).binSize_ns,
        0
    );
    setTimeWave(vector<double>(getNumTimeBins(0)), 0);
    setPeakIntensityIndex(WaveMaths::calcPropagationTimeLegacy(
        getTimeWave(0),
        getNumTimeBins(0),
        getFWFSettings(0).binSize_ns,
        getPulseLength_ns(0),
        7.0  // 3.5 too many ops., 7.0 just one op.
    ), 0);
}


double SingleScanner::calcFootprintArea(
    double const distance, size_t const idx
) const {
    return PI_QUARTER * distance * distance * getBt2(0);
}

double SingleScanner::calcTargetArea(
    double const distance, size_t const idx
) const{
    return calcFootprintArea(distance, 0) / ((double)getNumRays(0));
}

Rotation SingleScanner::calcAbsoluteBeamAttitude(size_t const idx) {
    return scanDev.calcAbsoluteBeamAttitude(
        platform->getAbsoluteMountAttitude()
    );
}
void SingleScanner::computeSubrays(
    std::function<void(
        vector<double> const &_tMinMax,
        int const circleStep,
        double const circleStep_rad,
        Rotation &r1,
        double const divergenceAngle,
        NoiseSource<double> &intersectionHandlingNoiseSource,
        std::map<double, double> &reflections,
        vector<RaySceneIntersection> &intersects
    )> handleSubray,
    vector<double> const &tMinMax,
    NoiseSource<double> &intersectionHandlingNoiseSource,
    std::map<double, double> &reflections,
    vector<RaySceneIntersection> &intersects,
    size_t const idx
){
    scanDev.computeSubrays(
        handleSubray,
        tMinMax,
        intersectionHandlingNoiseSource,
        reflections,
        intersects
    );
}

bool SingleScanner::initializeFullWaveform(
    double const minHitDist_m,
    double const maxHitDist_m,
    double &minHitTime_ns,
    double &maxHitTime_ns,
    double &nsPerBin,
    double &distanceThreshold,
    int &peakIntensityIndex,
    int &numFullwaveBins,
    size_t const idx
){
    return scanDev.initializeFullWaveform(
        minHitDist_m,
        maxHitDist_m,
        minHitTime_ns,
        maxHitTime_ns,
        nsPerBin,
        distanceThreshold,
        peakIntensityIndex,
        numFullwaveBins
    );
}

double SingleScanner::calcIntensity(
    double const incidenceAngle,
    double const targetRange,
    double const targetReflectivity,
    double const targetSpecularity,
    double const targetSpecularExponent,
    double const targetArea,
    double const radius,
    size_t const idx
) const {
    return scanDev.calcIntensity(
        incidenceAngle,
        targetRange,
        targetReflectivity,
        targetSpecularity,
        targetSpecularExponent,
        targetArea,
        radius
    );
}
double SingleScanner::calcIntensity(
    double const targetRange,
    double const radius,
    double const sigma,
    size_t const idx
) const{
    return scanDev.calcIntensity(targetRange, radius, sigma);
}
