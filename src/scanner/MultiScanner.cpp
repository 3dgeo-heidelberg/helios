#include <scanner/MultiScanner.h>
#include <maths/WaveMaths.h>
#include <scanner/detector/AbstractDetector.h>
#include <scanner/EvalScannerHead.h>
#include <scanner/beamDeflector/evaluable/EvalPolygonMirrorBeamDeflector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
MultiScanner::MultiScanner(MultiScanner &scanner) :
    Scanner(scanner),
    scanDevs(std::move(scanner.scanDevs))
{}


// ***   C L O N E   *** //
// ********************* //
std::shared_ptr<Scanner> MultiScanner::clone(){
    std::shared_ptr<Scanner> scanner = std::make_shared<MultiScanner>(*this);
    _clone(*scanner);
    return scanner;
}

void MultiScanner::_clone(Scanner &sc) const{
    // Call parent clone method
    Scanner::_clone(sc);
    // Clone attributes from MultiScanner class itself
    //MultiScanner &ssc = static_cast<MultiScanner &>(sc);  // Not used
    //ssc.scanDevs = scanDevs;  // Already handled by copy constructor
}

// ***  SIM STEP UTILS  *** //
// ************************ //
void MultiScanner::onLegComplete(){
    // Call parent handler for on leg complete events
    Scanner::onLegComplete();
    // Call each detector's handle method for on leg complete events
    size_t const nDevs = getNumDevices();
    for(size_t i = 0 ; i < nDevs ; ++i){
        getDetector(i)->onLegComplete();
    }
}

// ***   M E T H O D S   *** //
// ************************* //
void MultiScanner::prepareSimulation() {
    size_t const numDevs = getNumDevices();
    for(size_t i = 0 ; i < numDevs ; ++i){ // For each i-th device
        // Link the deflector angle with the evaluable scanner head
        std::shared_ptr<EvalScannerHead> sh =
            std::dynamic_pointer_cast<EvalScannerHead>(getScannerHead(i));
        std::shared_ptr<PolygonMirrorBeamDeflector> pmbd =
            std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(
                getBeamDeflector(i)
            );
        if(sh != nullptr && pmbd != nullptr){
            std::shared_ptr<EvalPolygonMirrorBeamDeflector> epmbd =
                std::dynamic_pointer_cast<EvalPolygonMirrorBeamDeflector>(
                    pmbd
                );
            if(epmbd != nullptr){
                sh->setDeflectorAnglePtr(
                    &epmbd->state_currentExactBeamAngle_rad
                );
            }
            else{
                sh->setDeflectorAnglePtr(&pmbd->state_currentBeamAngle_rad);
            }
        }
    }
}

void MultiScanner::applySettings(
    std::shared_ptr<ScannerSettings> settings, size_t const idx
){
    // Configure scanner
    setPulseFreq_Hz(settings->pulseFreq_Hz);
    setActive(settings->active);
    setBeamDivergence(settings->beamDivAngle, 0);
    trajectoryTimeInterval_ns = settings->trajectoryTimeInterval*1e9;

    // Configure scanning devices and their components
    size_t const numScanDevs = scanDevs.size();
    for(size_t i = 0 ; i < numScanDevs ; ++i){
        scanDevs[i].configureBeam();
        getDetector(i)->applySettings(settings);
        getScannerHead(i)->applySettings(settings);
        getBeamDeflector(i)->applySettings(settings);
    }
}

void MultiScanner::doSimStep(
    unsigned int legIndex, double const currentGpsTime
){
    // Check whether the scanner is active or not
    bool const _isActive = isActive();

    // Simulate scanning devices
    size_t const numScanDevs = getNumDevices();
    glm::dvec3 const absoluteMountPosition =
        platform->getAbsoluteMountPosition();
    Rotation const absoluteMountAttitude =
        platform->getAbsoluteMountAttitude();
    for(size_t i = 0 ; i < numScanDevs ; ++i){
        scanDevs[i].doSimStep(
            legIndex,
            currentGpsTime,
            cfg_setting_pulseFreq_Hz,
            _isActive,
            absoluteMountPosition,
            absoluteMountAttitude,
            [&] (glm::dvec3 &origin, Rotation &attitude) -> void {
                handleSimStepNoise(origin, attitude);
            },
            [&] (SimulatedPulse const &sp) -> void {
                spp->handlePulseComputation(sp);
            }
        );
    }

    // If the scanner is inactive, stop here:
    if (!_isActive) return;

    // Handle trajectory output
    handleTrajectoryOutput(currentGpsTime);
}

void MultiScanner::prepareDiscretization(size_t const idx){
    setNumTimeBins(
        getPulseLength_ns(idx) / getFWFSettings(idx).binSize_ns,
        idx
    );
    setTimeWave(vector<double>(getNumTimeBins(idx)), idx);
    setPeakIntensityIndex(WaveMaths::calcPropagationTimeLegacy(
        getTimeWave(idx),
        getNumTimeBins(idx),
        getFWFSettings(idx).binSize_ns,
        getPulseLength_ns(idx),
        7.0  // 3.5 too many ops., 7.0 just one op.
    ), idx);
}

double MultiScanner::calcFootprintArea(
    double const distance, size_t const idx
) const{
    return PI_QUARTER * distance * distance * getBt2(idx);
}

double MultiScanner::calcTargetArea(
    double const distance, size_t const idx
) const{
    return calcFootprintArea(distance, idx) / ((double)getNumRays(idx));
}

Rotation MultiScanner::calcAbsoluteBeamAttitude(size_t const idx){
    return scanDevs[idx].calcAbsoluteBeamAttitude(
        platform->getAbsoluteMountAttitude()
    );
}
void MultiScanner::computeSubrays(
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
    scanDevs[idx].computeSubrays(
        handleSubray,
        tMinMax,
        intersectionHandlingNoiseSource,
        reflections,
        intersects
    );
}

bool MultiScanner::initializeFullWaveform(
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
    return scanDevs[idx].initializeFullWaveform(
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

double MultiScanner::calcIntensity(
    double const incidenceAngle,
    double const targetRange,
    Material const &mat,
    double const targetArea,
    double const radius,
    size_t const idx
#ifdef DATA_ANALYTICS
    ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
) const{
    return scanDevs[idx].calcIntensity(
        incidenceAngle,
        targetRange,
        mat,
        targetArea,
        radius
#ifdef DATA_ANALYTICS
        ,pulseRecorder
#endif
    );
}

double MultiScanner::calcIntensity(
    double const targetRange,
    double const radius,
    double const sigma,
    size_t const idx
) const{
    return scanDevs[idx].calcIntensity(targetRange, radius, sigma);
}
