#include <Scanner.h>
#include <scanner/detector/AbstractDetector.h>
#include <filems/facade/FMSFacade.h>

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <logging.hpp>
#ifdef PYTHON_BINDING
#include "PyDetectorWrapper.h"
#endif

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
    cfg_setting_pulseFreq_Hz(pulseFreqs.front()),
    cfg_device_supportedPulseFreqs_Hz(pulseFreqs)
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
    this->numRays = s.numRays;
    this->cfg_setting_pulseFreq_Hz = s.cfg_setting_pulseFreq_Hz;
    this->state_currentPulseNumber = s.state_currentPulseNumber;
    this->state_lastPulseWasHit = s.state_lastPulseWasHit;
    this->state_isActive = s.state_isActive;
    this->numTimeBins = s.numTimeBins;
    this->peakIntensityIndex = s.peakIntensityIndex;

    this->fms = s.fms;
    if(s.scannerHead == nullptr) this->scannerHead = nullptr;
    else this->scannerHead = std::make_shared<ScannerHead>(*s.scannerHead);
    if(s.beamDeflector == nullptr) this->beamDeflector = nullptr;
    else this->beamDeflector = s.beamDeflector->clone();
    if(s.platform == nullptr) this->platform = nullptr;
    else this->platform = s.platform->clone();
    if(s.detector == nullptr) this->detector = nullptr;
    else this->detector = s.detector->clone();

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

    this->FWF_settings = FWFSettings(s.FWF_settings);
    this->time_wave = std::vector<double>(s.time_wave);

    this->cfg_device_supportedPulseFreqs_Hz = std::list<int>(
        s.cfg_device_supportedPulseFreqs_Hz
    );

    /*
     * Randomness generators must be initialized outside,
     * because DEFAULT_RG might not be instantiated at construction time
     */
    //initializeSequentialGenerators();
}

// ***  M E T H O D S  *** //
// *********************** //
void Scanner::applySettings(shared_ptr<ScannerSettings> settings) {
	// Configure scanner:
	this->setPulseFreq_Hz(settings->pulseFreq_Hz);
	setActive(settings->active);
	this->cfg_device_beamDivergence_rad = settings->beamDivAngle;
    trajectoryTimeInterval_ns = settings->trajectoryTimeInterval*1000000000.0;
    configureBeam();

	detector->applySettings(settings);
	scannerHead->applySettings(settings);
	beamDeflector->applySettings(settings);
}

std::shared_ptr<ScannerSettings> Scanner::retrieveCurrentSettings(){
    shared_ptr<ScannerSettings> settings = make_shared<ScannerSettings>();
    // Settings from Scanner
    std::stringstream ss;
    ss << id << "_settings";
    settings->id = ss.str();
    settings->pulseFreq_Hz = getPulseFreq_Hz();
    settings->active = isActive();
    settings->beamDivAngle = getBeamDivergence();
    settings->trajectoryTimeInterval = trajectoryTimeInterval_ns/1000000000.0;
    // Settings from ScannerHead
    settings->headRotatePerSec_rad = scannerHead->getRotateStart();
    settings->headRotateStart_rad = scannerHead->getRotateCurrent();
    settings->headRotateStop_rad = scannerHead->getRotateStop();
    // Settings from AbstractBeamDeflector
    settings->scanAngle_rad = beamDeflector->cfg_setting_scanAngle_rad;
    settings->scanFreq_Hz = beamDeflector->cfg_setting_scanFreq_Hz;
    settings->verticalAngleMin_rad = \
        beamDeflector->cfg_setting_verticalAngleMin_rad;
    settings->verticalAngleMax_rad = \
        beamDeflector->cfg_setting_verticalAngleMax_rad;
    // Return settings
    return settings;
}

void Scanner::applySettingsFWF(FWFSettings settings) {
	FWF_settings = settings;
	calcRaysNumber();
	prepareDiscretization();
}

string Scanner::toString() {
    std::stringstream ss;
    ss  << "Scanner: " << getScannerId() << "\n";
    size_t const numDevices = getNumDevices();
    for(size_t i = 0 ; i < numDevices ; ++i){
        ss  << "Device[" << i << "]: " << getDeviceId(i) << "\n"
            << "\tAverage Power: " << getAveragePower(i) << "W\n"
            << "\tBeam Divergence: " << getBeamDivergence(i)*1e3 << "mrad\n"
            << "\tWavelength: " << (int)(getWavelength(i)*1e9) << "nm\n"
            << "\tVisibility: " << getVisibility(i) << "km\n";
    }
    return ss.str();
}

void Scanner::doSimStep(
    unsigned int const legIndex,
    double currentGpsTime
) {
    // TODO Rethink : Make pure virtual, move implementation to SingleScanner
    // Update head attitude (we do this even when the scanner is inactive):
    scannerHead->doSimStep(cfg_setting_pulseFreq_Hz);

    // If the scanner is inactive, stop here:
    if (!isActive()) return;

    // Update beam deflector attitude:
    this->beamDeflector->doSimStep();

    // Check last pulse
    if (!beamDeflector->lastPulseLeftDevice()) return;

    // Global pulse counter:
    state_currentPulseNumber++;

    // Calculate absolute beam originWaypoint:
    glm::dvec3 absoluteBeamOrigin = platform->getAbsoluteMountPosition() +
        getHeadRelativeEmitterPosition();

	// Calculate absolute beam attitude:
	Rotation absoluteBeamAttitude = calcAbsoluteBeamAttitude();

    // Handle noise
    handleSimStepNoise(absoluteBeamOrigin, absoluteBeamAttitude);

	// Handle trajectory output
	handleTrajectoryOutput(currentGpsTime);

	// Pulse computation
    spp->handlePulseComputation(
        legIndex,
        absoluteBeamOrigin,
        absoluteBeamAttitude,
        currentGpsTime
    );
}


void Scanner::calcRaysNumber() {
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
	ss << "Number of subsampling rays: " << numRays;
	logging::INFO(ss.str());
}

void Scanner::prepareDiscretization(){
    numTimeBins = cfg_device_pulseLength_ns / FWF_settings.binSize_ns;
    time_wave = vector<double>(numTimeBins);
    peakIntensityIndex = calcTimePropagation(time_wave, numTimeBins);
}

int Scanner::calcTimePropagation(vector<double> & timeWave, int numBins){
    double const step = FWF_settings.binSize_ns;
    double const tau = (cfg_device_pulseLength_ns * 0.5) / 3.5;
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

double Scanner::calcFootprintArea(double distance) {
	double Bt2 = cached_Bt2;
	double R = distance;
	return (M_PI * R * R * Bt2) / 4;
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
	    cfg_device_supportedPulseFreqs_Hz.begin(),
	    cfg_device_supportedPulseFreqs_Hz.end(),
	    pulseFreq_Hz) == cfg_device_supportedPulseFreqs_Hz.end()
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

#ifdef PYTHON_BINDING
PyDetectorWrapper * Scanner::getPyDetectorWrapper(){
    return new PyDetectorWrapper(detector);
}
#endif

void Scanner::setLastPulseWasHit(bool value) {
	if (value == state_lastPulseWasHit) return;
	//TODO see https://www.codeproject.com/Articles/12362/A-quot-synchronized-quot-statement-for-C-like-in-J
    this->state_lastPulseWasHit = value;
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

Rotation Scanner::calcAbsoluteBeamAttitude(){
    Rotation mountRelativeEmitterAttitude =
        scannerHead->getMountRelativeAttitude()
            .applyTo(cfg_device_headRelativeEmitterAttitude);
    return platform->getAbsoluteMountAttitude()
        .applyTo(mountRelativeEmitterAttitude)
        .applyTo(beamDeflector->getEmitterRelativeAttitude());
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
        detector->cfg_device_accuracy_m
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
                detector,
                state_currentPulseNumber,
                writeWaveform,
                calcEchowidth,
                allMeasurements,
                allMeasurementsMutex,
                cycleMeasurements,
                cycleMeasurementsMutex,
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
                detector,
                state_currentPulseNumber,
                writeWaveform,
                calcEchowidth,
                allMeasurements,
                allMeasurementsMutex,
                cycleMeasurements,
                cycleMeasurementsMutex,
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
