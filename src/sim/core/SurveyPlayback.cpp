#include "logging.hpp"
#include <string>
#include <iomanip>

#include <memory>
#include <chrono>
using namespace std::chrono;

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "AbstractDetector.h"
#include "SurveyPlayback.h"
#include <glm/gtx/vector_angle.hpp>
#include "HelicopterPlatform.h"
#include <platform/InterpolatedMovingPlatform.h>
#include <ScanningStrip.h>
#include <filems/facade/FMSFacade.h>
#include <filems/write/comps/SimpleSyncFileWriter.h>

using helios::filems::SimpleSyncFileWriter;

using namespace std;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SurveyPlayback::SurveyPlayback(
    shared_ptr<Survey> survey,
    shared_ptr<FMSFacade> fms,
    int const parallelizationStrategy,
    std::shared_ptr<PulseThreadPoolInterface> pulseThreadPoolInterface,
    int const chunkSize,
    std::string fixedGpsTimeStart,
    bool const legacyEnergyModel,
    bool const exportToFile
):
    Simulation(
        parallelizationStrategy,
        pulseThreadPoolInterface,
        chunkSize,
        fixedGpsTimeStart,
        legacyEnergyModel
    ),
    fms(fms)
{
    this->mSurvey = survey;
    this->mSurvey->hatch(*this);
	this->exitAtEnd = true;
	this->exportToFile = exportToFile;
	this->setScanner(mSurvey->scanner);

    // Disable exports if requested
    if(!this->exportToFile){
        mSurvey->scanner->platform->writeNextTrajectory = false;
    }

    // ############### BEGIN If the leg has no survey defined, create a default one ################
	if (mSurvey->legs.size() == 0) {
		shared_ptr<Leg> leg(new Leg());

		// Set leg scanner settings:
		leg->mScannerSettings = make_shared<ScannerSettings>();

		// Set leg position to the center of the scene:
		shared_ptr<PlatformSettings> ps = make_shared<PlatformSettings>();
		ps->setPosition(
		    mSurvey->scanner->platform->scene->getAABB()->getCentroid()
        );
		leg->mPlatformSettings = ps;

		// Add leg to survey:
		mSurvey->addLeg(0, leg);
	}
    // ############### END If the leg has no survey defined, create a default one ################

    // If we start a new scan, move platform to destination of first leg:
    if(!mSurvey->legs.empty()) {
        getScanner()->platform->setPosition(
            mSurvey->legs[0]->mPlatformSettings->getPosition()
        );
    }

    // If platform is interpolated from data, sync GPS time if requested
    shared_ptr<InterpolatedMovingPlatform> imp =
        dynamic_pointer_cast<InterpolatedMovingPlatform>(
            getScanner()->platform
        );
    if(imp != nullptr && imp->isSyncGPSTime()){
        if(mSurvey->legs[0]->mTrajectorySettings->hasStartTime()){
            // Use start time of first leg
            this->currentGpsTime_ns = (
                mSurvey->legs[0]->mTrajectorySettings->tStart +
                imp->getStartTime()
            ) * 1000000000.0;
        }
        else{  // Use min time from input trajectory
            this->currentGpsTime_ns = imp->getStartTime() * 1000000000.0;
        }
    }

	// Orientate platform and start first leg
	startLeg(0, true);

    // For progress tracking
	numEffectiveLegs = mSurvey->legs.size();
    if(getScanner()->platform->canMove()) {
		mSurvey->calculateLength();
		numEffectiveLegs--;
	}
}


void SurveyPlayback::estimateTime(
    int legCurrentProgress,
    bool onGround,
    double legElapsedLength
){
    if (legCurrentProgress > legProgress) {	// Do stuff only if leg progress incremented at least 1%
		legProgress = (double) legCurrentProgress;

		chrono::nanoseconds currentTime = duration_cast<nanoseconds>(
		    system_clock::now().time_since_epoch());
		legElapsedTime_ns = currentTime - legStartTime_ns;
		legRemainingTime_ns = (long long)((100 - legProgress) / legProgress
		    * legElapsedTime_ns.count());

		if (
		        !getScanner()->platform->canMove() ||
		        getScanner()->platform->isInterpolated()
        ) {
			progress = ((mCurrentLegIndex * 100) + legProgress) /
			    (double) numEffectiveLegs;
		}
		else {
            progress = (elapsedLength + legElapsedLength) * 100
                / (double) mSurvey->getLength();
		}
		elapsedTime_ns = currentTime - timeStart_ns;
		remainingTime_ns = (long long)((100 - progress) / progress
            * elapsedTime_ns.count());

        if(legProgress == 99){
            std::stringstream ss;
            ss << "Final yaw = " <<
                mSurvey->scanner->platform->getHeadingRad();
            logging::TRACE(ss.str());

        }
        ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "Survey " << progress << "%\tElapsed "
            << milliToString(elapsedTime_ns.count()/1000000L) <<
            " Remaining " << milliToString(remainingTime_ns/1000000L) << "\n";
        oss << "Leg" << (mCurrentLegIndex+1) << "/" << (numEffectiveLegs)
            << " " << legProgress << "%\tElapsed "
            << milliToString(legElapsedTime_ns.count()/1000000L)
            << " Remaining " << milliToString(legRemainingTime_ns/1000000L);
        logging::INFO(oss.str());
	}
}

int SurveyPlayback::estimateSpatialLegProgress(double const legElapsedLength){
    return (int)(
        legElapsedLength * 100 / getCurrentLeg()->getLength()
    );
}
int SurveyPlayback::estimateAngularLegProgress(double const legElapsedAngle){
    return (int)(
        legElapsedAngle * 100 /
        getScanner()->getScannerHead()->getRotateRange()
    );
}
int SurveyPlayback::estimateTemporalLegProgress(){
    std::shared_ptr<InterpolatedMovingPlatform> imp =
        std::static_pointer_cast<InterpolatedMovingPlatform>(
            mSurvey->scanner->platform
        );
    double const t0 = imp->getCurrentLegStartTime();
    double const t = imp->getStepLoop().getCurrentTime();
    double const Dt = imp->getCurrentLegTimeDiff();
    return (int)(100*t/(t0 + Dt));
}

void SurveyPlayback::trackProgress() {
    std::shared_ptr<Platform> platform = getScanner()->platform;
	if(!platform->canMove()){
		double legElapsedAngle = std::fabs(
		    getScanner()->getScannerHead()->getRotateStart() -
		    getScanner()->getScannerHead()->getExactRotateCurrent()
        );
		int const legProgress = estimateAngularLegProgress(legElapsedAngle);
		estimateTime(legProgress, true, 0);
	}
	else if (mCurrentLegIndex < mSurvey->legs.size() - 1) {
        double const legElapsedLength = glm::distance(
            getCurrentLeg()->mPlatformSettings->getPosition(),
            mSurvey->scanner->platform->getPosition()
        );
        int const legProgress = platform->isInterpolated() ?
            estimateTemporalLegProgress() :
            estimateSpatialLegProgress(legElapsedLength);
        estimateTime(legProgress, false, legElapsedLength);
	}
}

void SurveyPlayback::doSimStep() {
	if (!mLegStarted) {
		mLegStarted = true;
		if(exportToFile) clearPointcloudFile();

		legProgress = 0;
		legStartTime_ns = duration_cast<nanoseconds>(
            system_clock::now().time_since_epoch()
        );
	}

	trackProgress();

	//if(legProgress <= 20) // Profiling only (uncomment for profiling)
    Simulation::doSimStep();
	//else onLegComplete(); // Profiling only (uncomment for profiling)
}

shared_ptr<Leg> SurveyPlayback::getCurrentLeg() {
	if (mCurrentLegIndex < mSurvey->legs.size()) {
		return mSurvey->legs.at(mCurrentLegIndex);
	}
	// NOTE: This should never happen:
	logging::ERR("ERROR getting current leg: Index out of bounds");
	return nullptr;
}

shared_ptr<Leg> SurveyPlayback::getPreviousLeg(){
    if(mCurrentLegIndex == 0) return nullptr;
    return mSurvey->legs.at(mCurrentLegIndex - 1);
}

int SurveyPlayback::getCurrentLegIndex() {
	return mCurrentLegIndex;
}

string SurveyPlayback::getLegOutputPrefix(std::string format){
    std::shared_ptr<Leg> leg = getCurrentLeg();
    std::shared_ptr<ScanningStrip> strip = leg->getStrip();

    // Mark leg as processed
    leg->wasProcessed = true;

    stringstream ss;
    if(strip != nullptr){ // Handle prefix when leg belongs to a split
        ss << "strip"
           << boost::str(boost::format(format) % strip->getStripId());
    }
    else{ // Handle prefix when leg does not belong to a split
        ss << "leg"
           << boost::str(boost::format(format) % leg->getSerialId());
    }
    return ss.str();
}

void SurveyPlayback::onLegComplete() {
    // Do scanning pulse process handling of on leg complete, if any
    mScanner->onLegComplete();

	// Start next leg
    elapsedLength += mSurvey->legs.at(mCurrentLegIndex)->getLength();
	startNextLeg(false);
}

void SurveyPlayback::startLeg(unsigned int const legIndex, bool const manual) {
	if (legIndex < 0 || legIndex >= mSurvey->legs.size()) {
		return;
	}

    ostringstream oss;oss << "Starting leg " << legIndex << endl;
	logging::INFO(oss.str());
	mLegStarted = false;
	mCurrentLegIndex = legIndex;
	shared_ptr<Leg> leg = getCurrentLeg();

	// Apply scanner settings:
    if (leg->mScannerSettings != nullptr) {
		mSurvey->scanner->applySettings(leg->mScannerSettings);
	}
	shared_ptr<Platform> platform(getScanner()->platform);
    mSurvey->scanner->lastTrajectoryTime = 0L;

	// Apply platform settings:
    if (leg->mPlatformSettings != nullptr) {
		platform->applySettings(leg->mPlatformSettings, manual);

		// ################ BEGIN Set platform destination ##################
		unsigned int nextLegIndex = legIndex + 1;

		if (nextLegIndex < mSurvey->legs.size()) {
			// Set destination to position of next leg:
			shared_ptr<Leg> nextLeg = mSurvey->legs.at(nextLegIndex);
            if(
                leg->mTrajectorySettings != nullptr &&
                leg->mTrajectorySettings->teleportToStart
            ){
                platform->setPosition(
                    nextLeg->mPlatformSettings->getPosition()
                );
                platform->setOrigin(
                    nextLeg->mPlatformSettings->getPosition()
                );
                platform->setDestination(
                    nextLeg->mPlatformSettings->getPosition()
                );
            }
            else{
                platform->setOrigin(leg->mPlatformSettings->getPosition());
                if(
                    nextLeg->mTrajectorySettings != nullptr &&
                    nextLeg->mTrajectorySettings->teleportToStart
                ){
                    // If next leg teleports to start, current leg is stop leg
                    // Thus, set stop origin and destination to the same point
                    platform->setDestination(
                        leg->mPlatformSettings->getPosition()
                    );
                }
                else{
                    platform->setDestination(
                        nextLeg->mPlatformSettings->getPosition()
                    );
                }
            }
            if(nextLegIndex + 1 < mSurvey->legs.size()){
                platform->setAfterDestination(
                    mSurvey->legs.at(nextLegIndex + 1)
                        ->mPlatformSettings->getPosition()
                );
            }
            else{
                platform->setAfterDestination(
                    nextLeg->mPlatformSettings->getPosition()
                );
            }
            stringstream ss;
            ss  << "Leg" << legIndex << " waypoints:\n"
                << "\tOrigin: (" << platform->originWaypoint.x << ", "
                    << platform->originWaypoint.y << ", "
                    << platform->originWaypoint.z << ")\n"
                << "\tTarget: (" << platform->targetWaypoint.x << ", "
                    << platform->targetWaypoint.y << ", "
                    << platform->targetWaypoint.z << ")\n"
                << "\tNext: (" << platform->nextWaypoint.x << ", "
                    << platform->nextWaypoint.y << ", "
                    << platform->nextWaypoint.z << ")" << std::endl;
            logging::INFO(ss.str());
		}

        if(platform->canStopAndTurn() && leg->mPlatformSettings->stopAndTurn)
            stopAndTurn(legIndex, leg);
		else if(manual) platform->initLegManual();
        else platform->initLeg();
        try{ // Transform trajectory time (if any) to simulation time
            if(
                leg->mTrajectorySettings != nullptr &&
                leg->mTrajectorySettings->hasStartTime()
            ){
                std::shared_ptr<InterpolatedMovingPlatform> imp =
                    dynamic_pointer_cast<InterpolatedMovingPlatform>(platform);
                imp->toTrajectoryTime(leg->mTrajectorySettings->tStart);
            }
        }catch(...) {}
        // Set the interpolated moving platform time difference from target leg
        if(platform->isInterpolated()){
            std::shared_ptr<InterpolatedMovingPlatform> imp =
                dynamic_pointer_cast<InterpolatedMovingPlatform>(platform);
            if(!(
                leg->mTrajectorySettings->hasStartTime() ||
                leg->mTrajectorySettings->hasEndTime()
            )){  // If stop leg
                imp->setCurrentLegTimeDiff(0);
            }
            else{  // If non-stop leg
                imp->setCurrentLegTimeDiff(
                    leg->mTrajectorySettings->tEnd -
                    leg->mTrajectorySettings->tStart
                );
            }
        }

		// ################ END Set platform destination ##################
		platform->prepareLeg(mScanner->getPulseFreq_Hz());
	}

    // Restart deflector if previous leg was not active
	shared_ptr<Leg> previousLeg = getPreviousLeg();
    if(
	    previousLeg != nullptr && !previousLeg->mScannerSettings->active &&
	    leg->mScannerSettings->active
    ){
        mSurvey->scanner->getBeamDeflector()->restartDeflector();
	}


    if(exportToFile){
        prepareOutput();
        platform->writeNextTrajectory = true;
    }
}

void SurveyPlayback::startNextLeg(bool manual) {
	if (mCurrentLegIndex < mSurvey->legs.size() - 1) {
		// If there are pending legs, start the next one:
		startLeg(mCurrentLegIndex + 1, manual);
	}
    else if(simPlayer->hasPendingPlays()){
        simPlayer->prepareRepeat();
    }
    else if(exitAtEnd) {
        // If this was the final leg, stop the simulation:
        shutdown();
        stop();
    }
    else {
        pause(true);
    }
}

void SurveyPlayback::shutdown() {
	Simulation::shutdown();
	mSurvey->scanner->getDetector()->shutdown();
}

string SurveyPlayback::milliToString(long millis) {
	long seconds = (millis / 1000) % 60;
	long minutes = (millis / (60000)) % 60;
	long hours = (millis / (3600000)) % 24;
	long days = millis / 86400000;
	return boost::str(
	    boost::format("%02d %02d:%02d:%02d") % days % hours % minutes % seconds
    );
}

void SurveyPlayback::stopAndTurn(
    unsigned int legIndex,
    std::shared_ptr<Leg> leg
){
    if(legIndex == mSurvey->legs.size()-1) return;
    glm::dvec3 currentPos = leg->mPlatformSettings->getPosition();
    glm::dvec3 nextPos =
        mSurvey->legs[legIndex+1]->mPlatformSettings->getPosition();
    glm::dvec3 xyDir = nextPos-currentPos;
    xyDir.z = 0;
    xyDir = glm::normalize(xyDir);
    glm::dvec3 xyRef = glm::dvec3(0,1,0);
    double sign = (xyDir.x >= 0) ? 1 : -1;
    // sign*acos(-dotProduct/magnitudeProduct)
    double angle = sign*std::acos(
        -glm::dot(xyRef,xyDir) / glm::l2Norm(xyRef) / glm::l2Norm(xyDir)
    );

    mSurvey->scanner->platform->setHeadingRad(angle);
    HelicopterPlatform *hp = dynamic_cast<HelicopterPlatform*>(
        mSurvey->scanner->platform.get());
    if(hp!=nullptr){
        std::shared_ptr<PlatformSettings> lps = leg->mPlatformSettings;
        if(lps->yawAtDepartureSpecified) hp->yaw = lps->yawAtDeparture;
        hp->pitch = 0.0;
        hp->speed_xy.x = 0.0001;
        hp->speed_xy.y = 0.0001;
    }
    std::stringstream ss;
    ss << "stop and turn yaw = " << angle;
    logging::TRACE(ss.str());
}

void SurveyPlayback::prepareOutput(){
    // Check File Management System is active (sim without output is possible)
    if(fms == nullptr) return;

    // Check leg is active, otherwise there is no output to prepare
    std::shared_ptr<Leg> leg = getCurrentLeg();
    if(!leg->getScannerSettings().active) return;

    // Check if all the legs in the strip were processed
    std::shared_ptr<ScanningStrip> strip = leg->getStrip();
    bool lastLegInStrip = true;
    if(strip != nullptr){
        lastLegInStrip = getCurrentLeg()->getStrip()->isLastLegInStrip();
    }

    // Configure output paths
    fms->write.configure(
        getLegOutputPrefix(),
        getScanner()->isWriteWaveform(),
        getScanner()->isWritePulse(),
        lastLegInStrip
    );

    // Handle historical tracking of output paths
    getScanner()->trackOutputPath(
        fms->write.getMeasurementWriterOutputPath().string()
    );
}

void SurveyPlayback::clearPointcloudFile(){
    // Dont clear strip file, it would overwrite previous point cloud content
    if(getCurrentLeg()->isContainedInAStrip()) return;
    // Dont clear pcloud file, if leg is not active there is nothing to clear
    // Otherwise, WATCHOUT because last active leg might be overwriten
    if(!getCurrentLeg()->mScannerSettings->active) return;
    fms->write.clearPointcloudFile();
}


