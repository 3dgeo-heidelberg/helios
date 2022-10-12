#include <Scanner.h>
#include <scanner/detector/AbstractDetector.h>
#include <filems/facade/FMSFacade.h>

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <logging.hpp>

#include <scanner/BuddingScanningPulseProcess.h>
#include <scanner/WarehouseScanningPulseProcess.h>
#include <PulseTaskDropper.h>
#include <PulseThreadPool.h>
#include <Trajectory.h>

using namespace std;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Scanner::Scanner(
    std::string const id,
    std::list<int> const &pulseFreqs,
    bool const writeWaveform,
    bool const calcEchowidth,
    bool const fullWaveNoise,
    bool const platformNoiseDisabled
) :
    id(id),
    writeWaveform(writeWaveform),
    calcEchowidth(calcEchowidth),
    fullWaveNoise(fullWaveNoise),
    platformNoiseDisabled(platformNoiseDisabled),
    cfg_setting_pulseFreq_Hz(pulseFreqs.front())
{
    /*
     * Randomness generators must be initialized outside,
     * because DEFAULT_RG might not be instantiated at construction time
     */
    //initializeSequentialGenerators();
}

Scanner::Scanner(Scanner &s){
    this->writeWaveform = s.writeWaveform;
    this->calcEchowidth = s.calcEchowidth;
    this->fullWaveNoise = s.fullWaveNoise;
    this->platformNoiseDisabled = s.platformNoiseDisabled;
    this->cfg_setting_pulseFreq_Hz = s.cfg_setting_pulseFreq_Hz;
    this->state_isActive = s.state_isActive;

    this->fms = s.fms;
    if(s.platform == nullptr) this->platform = nullptr;
    else this->platform = s.platform->clone();

    if(s.allOutputPaths == nullptr) this->allOutputPaths = nullptr;
    else{
        this->allOutputPaths = std::make_shared<std::vector<std::string>>(
            *s.allOutputPaths
        );
    }
    if(s.allMeasurements == nullptr) this->allMeasurements = nullptr;
    else {
        this->allMeasurements = std::make_shared<std::vector<Measurement>>(
            *s.allMeasurements
        );
    }
    if(s.allTrajectories == nullptr) this->allTrajectories = nullptr;
    else{
        this->allTrajectories = std::make_shared<std::vector<Trajectory>>(
            *s.allTrajectories
        );
    }
    if(s.allMeasurementsMutex == nullptr) this->allMeasurementsMutex = nullptr;
    else this->allMeasurementsMutex = std::make_shared<std::mutex>();

    if(s.cycleMeasurements == nullptr) this->cycleMeasurements = nullptr;
    else {
        this->cycleMeasurements = std::make_shared<std::vector<Measurement>>(
            *s.cycleMeasurements
        );
    }
    if(s.cycleTrajectories == nullptr) this->cycleTrajectories = nullptr;
    else{
        this->cycleTrajectories = std::make_shared<std::vector<Trajectory>>(
            *s.cycleTrajectories
        );
    }
    if(s.cycleMeasurementsMutex == nullptr) this->cycleMeasurementsMutex = nullptr;
    else this->cycleMeasurementsMutex = std::make_shared<std::mutex>();

    /*
     * Randomness generators must be initialized outside,
     * because DEFAULT_RG might not be instantiated at construction time
     */
    //initializeSequentialGenerators();
}


// ***   C L O N E   *** //
// ********************* //
void Scanner::_clone(Scanner &sc) const{
    // Clone scanner attributes
    sc.id = id;
    sc.writeWaveform = writeWaveform;
    sc.calcEchowidth = calcEchowidth;
    sc.fullWaveNoise = fullWaveNoise;
    sc.platformNoiseDisabled = platformNoiseDisabled;
    sc.fixedIncidenceAngle = fixedIncidenceAngle;
    sc.cfg_setting_pulseFreq_Hz = cfg_setting_pulseFreq_Hz;
    sc.state_isActive = state_isActive;
    sc.spp = nullptr;  // Cannot be cloned (unique pointer)
    sc.fms = fms;
    if(platform == nullptr){
        sc.platform = nullptr;
    }
    else{
        sc.platform = platform->clone();
    }
    if(allOutputPaths == nullptr){
        sc.allOutputPaths = nullptr;
    }
    else{
        sc.allOutputPaths = std::make_shared<std::vector<std::string>>(
            *allOutputPaths
        );
    }
    if(allMeasurements == nullptr){
        sc.allMeasurements = nullptr;
    }
    else{
        sc.allMeasurements = std::make_shared<std::vector<Measurement>>(
            *allMeasurements
        );
    }
    if(allTrajectories == nullptr){
        sc.allTrajectories = nullptr;
    }
    else{
        sc.allTrajectories = std::make_shared<std::vector<Trajectory>>(
            *allTrajectories
        );
    }
    if(allMeasurementsMutex == nullptr){
        sc.allMeasurementsMutex = nullptr;
    }
    else{
        sc.allMeasurementsMutex = std::make_shared<std::mutex>();
    }
    if(cycleMeasurements == nullptr){
        sc.cycleMeasurements = nullptr;
    }
    else{
        sc.cycleMeasurements = std::make_shared<std::vector<Measurement>>(
            *cycleMeasurements
        );
    }
    if(cycleTrajectories == nullptr){
        sc.cycleTrajectories = nullptr;
    }
    else{
        sc.cycleTrajectories = std::make_shared<std::vector<Trajectory>>(
            *cycleTrajectories
        );
    }
    if(cycleMeasurementsMutex == nullptr){
        sc.cycleMeasurementsMutex = nullptr;
    }
    else{
        sc.cycleMeasurementsMutex = std::make_shared<std::mutex>();
    }
    sc.trajectoryTimeInterval_ns = trajectoryTimeInterval_ns;
    sc.lastTrajectoryTime = lastTrajectoryTime;
    if(randGen1 == nullptr){
        sc.randGen1 = nullptr;
    }
    else{
        sc.randGen1 = std::make_shared<RandomnessGenerator<double>>(*randGen1);
    }
    if(randGen2 == nullptr){
        sc.randGen2 = nullptr;
    }
    else{
        sc.randGen2 = std::make_shared<RandomnessGenerator<double>>(*randGen2);
    }
    if(intersectionHandlingNoiseSource == nullptr){
        sc.intersectionHandlingNoiseSource = nullptr;
    }
    else{
        sc.intersectionHandlingNoiseSource = std::make_shared<
            UniformNoiseSource<double>
        >(*intersectionHandlingNoiseSource);
    }
}


// ***  M E T H O D S  *** //
// *********************** //
std::shared_ptr<ScannerSettings> Scanner::retrieveCurrentSettings(
    size_t const idx
){
    shared_ptr<ScannerSettings> settings = make_shared<ScannerSettings>();
    // Settings from Scanner
    std::stringstream ss;
    ss << id << "_settings";
    settings->id = ss.str();
    settings->pulseFreq_Hz = getPulseFreq_Hz();
    settings->active = isActive();
    settings->beamDivAngle = getBeamDivergence(idx);
    settings->trajectoryTimeInterval = trajectoryTimeInterval_ns/1000000000.0;
    // Settings from ScannerHead
    settings->headRotatePerSec_rad = getScannerHead(idx)->getRotateStart();
    settings->headRotateStart_rad = getScannerHead(idx)->getRotateCurrent();
    settings->headRotateStop_rad = getScannerHead(idx)->getRotateStop();
    // Settings from AbstractBeamDeflector
    settings->scanAngle_rad = getBeamDeflector(idx)->cfg_setting_scanAngle_rad;
    settings->scanFreq_Hz = getBeamDeflector(idx)->cfg_setting_scanFreq_Hz;
    settings->verticalAngleMin_rad = \
        getBeamDeflector(idx)->cfg_setting_verticalAngleMin_rad;
    settings->verticalAngleMax_rad = \
        getBeamDeflector(idx)->cfg_setting_verticalAngleMax_rad;
    // Return settings
    return settings;
}

void Scanner::applySettingsFWF(FWFSettings settings, size_t const idx) {
    setFWFSettings(settings, idx);
	calcRaysNumber();
	prepareDiscretization();
}

string Scanner::toString() {
    std::stringstream ss;
    ss  << "Scanner: " << getScannerId() << "\n";
    size_t const numDevices = getNumDevices();
    for(size_t i = 0 ; i < numDevices ; ++i){
        ss  << "Device[" << i << "]: " << getDeviceId(i) << "\n"
            << "\tAverage Power: " << getAveragePower(i) << " W\n"
            << "\tBeam Divergence: " << getBeamDivergence(i)*1e3 << " mrad\n"
            << "\tWavelength: " << (int)(getWavelength(i)*1e9) << " nm\n"
            << "\tVisibility: " << getVisibility(i) << " km\n";
    }
    return ss.str();
}


double Scanner::calcFootprintRadius(double distance) {
	double area = calcFootprintArea(distance);
	return sqrt(area / M_PI);
}


void Scanner::setPulseFreq_Hz(int pulseFreq_Hz) {

	// Check of requested pulse freq is > 0:
	if (pulseFreq_Hz < 0) {
		logging::WARN(
		    "ERROR: Attempted to set pulse frequency < 0. "
            "This is not possible."
        );
		pulseFreq_Hz = 0;
	}

	// Check if requested pulse freq is supported by device:
	if(std::find(
	        getSupportedPulseFreqs_Hz().begin(),
	        getSupportedPulseFreqs_Hz().end(),
	        pulseFreq_Hz
        ) == getSupportedPulseFreqs_Hz().end()
    ){
		logging::WARN(
		    "WARNING: Specified pulse frequency is not supported "
            "by this device. We'll set it nevertheless.\n"
        );
	}

	// Set new pulse frequency:
	cfg_setting_pulseFreq_Hz = pulseFreq_Hz;

	stringstream ss;
	ss << "Pulse frequency set to " << this->cfg_setting_pulseFreq_Hz;
	logging::INFO(ss.str());
}

// ***  SIM STEP UTILS  *** //
// ************************ //
void Scanner::handleSimStepNoise(
    glm::dvec3 & absoluteBeamOrigin,
    Rotation & absoluteBeamAttitude
){
    // Apply noise to beam origin
    if(!platformNoiseDisabled){
        if (platform->positionXNoiseSource != nullptr){ // Add X position noise
            absoluteBeamOrigin.x += platform->positionXNoiseSource->next();
        }
        if (platform->positionYNoiseSource != nullptr){ // Add Y position noise
            absoluteBeamOrigin.y += platform->positionYNoiseSource->next();
        }
        if (platform->positionZNoiseSource != nullptr){ // Add Z position noise
            absoluteBeamOrigin.z += platform->positionZNoiseSource->next();
        }
    }

    // Apply noise to beam attitude
    if(!platformNoiseDisabled){
        if(platform->attitudeXNoiseSource != nullptr) { // Add attitude X noise
            absoluteBeamAttitude = absoluteBeamAttitude.applyTo(Rotation(
                Directions::right, platform->attitudeXNoiseSource->next()
            ));
        }
        if(platform->attitudeYNoiseSource != nullptr) { // Add attitude Y noise
            absoluteBeamAttitude = absoluteBeamAttitude.applyTo(Rotation(
                Directions::forward, platform->attitudeYNoiseSource->next()
            ));
        }
        if(platform->attitudeZNoiseSource != nullptr) {// Add attitude Z noise
            absoluteBeamAttitude = absoluteBeamAttitude.applyTo(Rotation(
                Directions::up, platform->attitudeZNoiseSource->next()
            ));
        }
    }
}

void Scanner::handleTrajectoryOutput(double const currentGpsTime){
    // Get out of here if trajectory time interval is 0 (no trajectory output)
    if(trajectoryTimeInterval_ns == 0.0) return;
    // Get out of here it it has been explicitly specified to dont write
    if(!platform->writeNextTrajectory) return;

    // Check elapsed time
    double const elapsedTime = currentGpsTime-lastTrajectoryTime;
    if(lastTrajectoryTime != 0L && elapsedTime < trajectoryTimeInterval_ns)
        return;

    // Update last trajectory time
    lastTrajectoryTime = currentGpsTime;

    // Compute shifted position
    glm::dvec3 const pos = platform->position + platform->scene->getShift();

    // Obtain roll, pitch and yaw
    double roll, pitch, yaw;
    platform->getRollPitchYaw(roll, pitch, yaw);
    //if(roll < 0.0) roll += PI_2; // Roll from [-pi, pi] to [0, 2pi]
    if(yaw < 0.0) yaw += PI_2;

    // Build trajectory
    Trajectory trajectory(currentGpsTime, pos, roll, pitch, yaw);

    // Write trajectory output
    fms->write.writeTrajectoryUnsafe(trajectory);

    // Add trajectory to all trajectories vector
    if(allTrajectories != nullptr){
        std::unique_lock<std::mutex> lock(*allMeasurementsMutex);
        allTrajectories->push_back(trajectory);
    }

    // Add trajectory to cycle trajectories vector
    if(cycleTrajectories != nullptr){
        std::unique_lock<std::mutex> lock(*cycleMeasurementsMutex);
        cycleTrajectories->push_back(trajectory);
    }

    // Avoid repeating trajectory for non moving platforms
    if(!platform->canMove()) platform->writeNextTrajectory = false;
}

void Scanner::trackOutputPath(std::string const &path){
    if(allOutputPaths != nullptr){
        std::unique_lock<std::mutex> lock(*allMeasurementsMutex);
        allOutputPaths->push_back(path);
    }
}

void Scanner::initializeSequentialGenerators(){
    randGen1 = std::make_shared<RandomnessGenerator<double>>(
        RandomnessGenerator<double>(*DEFAULT_RG));
    randGen2 = std::make_shared<RandomnessGenerator<double>>(
        RandomnessGenerator<double>(*DEFAULT_RG));
    randGen1->computeUniformRealDistribution(0.0, 1.0);
    randGen1->computeNormalDistribution(
        0.0,
        getDetector()->cfg_device_accuracy_m
    );
    randGen2->computeNormalDistribution(0.0, 1.0);
    intersectionHandlingNoiseSource =
        std::make_shared<UniformNoiseSource<double>>(
            *DEFAULT_RG, 0.0, 1.0
        );
    intersectionHandlingNoiseSource->configureUniformNoise(0.0, 1.0);
}

void Scanner::buildScanningPulseProcess(
    int const parallelizationStrategy,
    PulseTaskDropper &dropper,
    std::shared_ptr<PulseThreadPoolInterface> pool
){
    if(parallelizationStrategy==0){
        spp = std::unique_ptr<ScanningPulseProcess>(
            new BuddingScanningPulseProcess(
                getDetector(0)->scanner,
                dropper,
                *(std::static_pointer_cast<PulseThreadPool>(pool)),
                *randGen1,
                *randGen2,
                *intersectionHandlingNoiseSource
            )
        );
    }
    else if(parallelizationStrategy==1){
        spp = std::unique_ptr<ScanningPulseProcess>(
            new WarehouseScanningPulseProcess(
                getDetector(0)->scanner,
                dropper,
                *(std::static_pointer_cast<PulseWarehouseThreadPool>(pool)),
                *randGen1,
                *randGen2,
                *intersectionHandlingNoiseSource
            )
        );
    }
    else{
        std::stringstream ss;
        ss  << "Scanner::buildScanningPulseProcess unexpected parallelization "
            << "strategy: " << parallelizationStrategy;
        throw HeliosException(ss.str());
    }
}
