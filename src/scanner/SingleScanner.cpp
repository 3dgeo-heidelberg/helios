#include <scanner/SingleScanner.h>
#include <scanner/detector/AbstractDetector.h>

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
    ), // TODO Rethink : Pass properly arguments to parent constructor
    scanDev(
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
        [&] (
            unsigned int legIndex,
            glm::dvec3 &absoluteBeamOrigin,
            Rotation &absoluteBeamAttitude,
            double const currentGpsTime
        ) -> void{
            spp->handlePulseComputation(
                legIndex,
                absoluteBeamOrigin,
                absoluteBeamAttitude,
                currentGpsTime
            );
        }
    );

    // If the scanner is inactive, stop here:
    if (!_isActive) return;

    // Global pulse counter:
    ++state_currentPulseNumber;

    // Handle trajectory output
    handleTrajectoryOutput(currentGpsTime);
}

void SingleScanner::prepareDiscretization(){
    setNumTimeBins(
        getPulseLength_ns(0) / getFWFSettings(0).binSize_ns,
        0
    );
    setTimeWave(vector<double>(getNumTimeBins(0)), 0);
    setPeakIntensityIndex(calcTimePropagation(
        getTimeWave(0), getNumTimeBins(0)
    ), 0);
}

int SingleScanner::calcTimePropagation(
    std::vector<double> & timeWave, int const numBins
) {
    double const step = getFWFSettings(0).binSize_ns;
    //double const tau = (getPulseLength_ns() * 0.5) / 3.5;  // Too many ops.
    double const tau = getPulseLength_ns(0) / 7.0;  // Just one op.
    double t = 0;
    double t_tau = 0;
    double pt = 0;
    double peakValue = 0;
    int peakIndex = 0;

    for (int i = 0; i < numBins; ++i) {
        t = i * step;
        t_tau = t / tau;
        pt = (t_tau * t_tau) * exp(-t_tau);
        timeWave[i] = pt;
        if (pt > peakValue) {
            peakValue = pt;
            peakIndex = i;
        }
    }

    return peakIndex;
}

double SingleScanner::calcFootprintArea(
    double const distance, size_t const idx
) const {
    return PI_QUARTER * distance * distance * getBt2(0);
}

Rotation SingleScanner::calcAbsoluteBeamAttitude() {
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
