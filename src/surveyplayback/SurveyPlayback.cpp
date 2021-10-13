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
#include "FullWaveformPulseDetector.h"
#include "SurveyPlayback.h"
#include <glm/gtx/vector_angle.hpp>
#include "ZipSyncFileWriter.h"
#include "SimpleSyncFileWriter.h"
#include "HelicopterPlatform.h"

using namespace std;

SurveyPlayback::SurveyPlayback(
    shared_ptr<Survey> survey,
    const std::string outputPath,
    size_t numThreads,
    bool lasOutput,
    bool las10,
    bool zipOutput,
    bool exportToFile
):
    Simulation(numThreads, survey->scanner->detector->cfg_device_accuracy_m),
    lasOutput(lasOutput),
    las10(las10),
    zipOutput(zipOutput),
    outputPath(outputPath)
{
    this->mSurvey = survey;
	this->exitAtEnd = true;
	this->exportToFile = exportToFile;

	// ######## BEGIN Create part of the leg point cloud file path #######
	auto t = std::time(nullptr);
	auto tm = std::localtime(&t);
	auto mDateString = std::put_time(tm, "%Y-%m-%d_%H-%M-%S");
	ostringstream oss;
	oss << outputPath << "Survey Playback/" << mSurvey->name << "/"
	    << mDateString << "/";
	mOutputFilePathString = oss.str();
	logging::INFO(mOutputFilePathString);

	// ######## END Create part of the leg point cloud file path #######
	this->setScanner(mSurvey->scanner);

	// ############### BEGIN If the leg has no survey defined, create a default one ################
	if (mSurvey->legs.size() == 0) {
		shared_ptr<Leg> leg(new Leg());

		// Set leg scanner settings:
		leg->mScannerSettings = shared_ptr<ScannerSettings>(new ScannerSettings());

		// Set leg position to the center of the scene:
		shared_ptr<PlatformSettings> ps(new PlatformSettings());
		ps->setPosition(mSurvey->scanner->platform->scene->getAABB()->getCentroid());

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

	// Orientate platform and start first leg
	startLeg(0, true);

	// For progress tracking
	numEffectiveLegs = mSurvey->legs.size();
    if(getScanner()->platform->canMove()) {
		mSurvey->calculateLength();
		numEffectiveLegs--;
	}
    this->mSurvey = survey;
}


void SurveyPlayback::estimateTime(int legCurrentProgress, bool onGround, double legElapsedLength) {
    if (legCurrentProgress > legProgress) {	// Do stuff only if leg progress incremented at least 1%
		legProgress = (double) legCurrentProgress;

		long currentTime = duration_cast<milliseconds>(
		    system_clock::now().time_since_epoch()).count();
		legElapsedTime_ms = currentTime - legStartTime_ns;
		legRemainingTime_ms = (long)((100 - legProgress) / legProgress
		    * legElapsedTime_ms);

		if (!getScanner()->platform->canMove()) {
			progress = ((mCurrentLegIndex * 100) + legProgress) /
			    (double) numEffectiveLegs;
		}
		else {
			progress = (elapsedLength + legElapsedLength) * 100
			    / (double) mSurvey->getLength();
		}
		elapsedTime_ms = currentTime - timeStart_ms;
		remainingTime_ms = (long)((100 - progress) /
		    progress * elapsedTime_ms);

        if(legProgress == 99){
            std::stringstream ss;
            ss << "Final yaw = " <<
                mSurvey->scanner->platform->getHeadingRad();
            logging::TRACE(ss.str());

        }
        ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "Survey " << progress << "%\tElapsed "
            << milliToString(elapsedTime_ms) <<
            " Remaining " << milliToString(remainingTime_ms) << endl;
        oss << "Leg" << (mCurrentLegIndex+1) << "/" << (numEffectiveLegs)
            << " " << legProgress << "%\tElapsed "
            << milliToString(legElapsedTime_ms) << " Remaining "
            << milliToString(legRemainingTime_ms);
        logging::INFO(oss.str());
	}
}

void SurveyPlayback::trackProgress() {
	if(!getScanner()->platform->canMove()){
		double legElapsedAngle = std::fabs(
		    getScanner()->scannerHead->getRotateStart() -
		    getScanner()->scannerHead->getRotateCurrent()
        );
		int legProgress = (int)(legElapsedAngle * 100 / getScanner()->scannerHead->getRotateRange());
		estimateTime(legProgress, true, 0);
	}
	else if (mCurrentLegIndex < mSurvey->legs.size() - 1) {
		double legElapsedLength = glm::distance(
		    getCurrentLeg()->mPlatformSettings->getPosition(),
		    mSurvey->scanner->platform->getPosition()
        );
		int legProgress = (int)
		    (legElapsedLength * 100 / getCurrentLeg()->getLength());
		estimateTime(legProgress, false, legElapsedLength);
	}
}

void SurveyPlayback::doSimStep() {
	if (!mLegStarted) {
		mLegStarted = true;
		if(exportToFile) clearPointcloudFile();

		legProgress = 0;
		legStartTime_ns = duration_cast<milliseconds>(
		        system_clock::now().time_since_epoch()
        ).count();
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
	return NULL;
}

int SurveyPlayback::getCurrentLegIndex() {
	return mCurrentLegIndex;
}

string SurveyPlayback::getLegOutputPrefix(){
    stringstream ss;
    ss << "points/leg"
       << boost::str(boost::format(mFormatString) % getCurrentLegIndex());
    return ss.str();
}

string SurveyPlayback::getCurrentOutputPath() {
    stringstream ss;
    ss << getLegOutputPrefix();
	if(lasOutput){
	    if(zipOutput) ss << "_point.laz";
	    else ss << "_points.las";
	}
	else if(zipOutput) ss << "_points.bin";
	else ss << "_points.xyz";
	return ss.str();
}

string SurveyPlayback::getTrajectoryOutputPath(){
    stringstream ss;
    ss << getLegOutputPrefix();
    ss << "_trajectory.txt";
    return ss.str();
}

void SurveyPlayback::onLegComplete() {
	// Wait for threads to finish
	threadPool.join();

	// Start next leg
    elapsedLength += mSurvey->legs.at(mCurrentLegIndex)->getLength();
	startNextLeg(false);
}

void SurveyPlayback::startLeg(unsigned int legIndex, bool manual) {
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
			platform->setOrigin(leg->mPlatformSettings->getPosition());
            platform->setDestination(
                nextLeg->mPlatformSettings->getPosition()
            );
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

		// ################ END Set platform destination ##################
	}

	if(exportToFile) prepareOutput();
    platform->writeNextTrajectory = true;
}

void SurveyPlayback::startNextLeg(bool manual) {
	if (mCurrentLegIndex < mSurvey->legs.size() - 1) {
		// If there are still legs left, start the next one:
		startLeg(mCurrentLegIndex + 1, manual);
	}
	else {
		// If this was the final leg, stop the simulation:
		if (exitAtEnd) {
			shutdown();
			stop();
		}
		else {
			pause(true);
		}
	}
}

void SurveyPlayback::shutdown() {
	Simulation::shutdown();
	mSurvey->scanner->detector->shutdown();
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
    // Specify output file:
    shared_ptr<FullWaveformPulseDetector> fwf_detector =
        dynamic_pointer_cast<FullWaveformPulseDetector>(
            getScanner()->detector
        );
    stringstream ss;
    ss << "leg"
       << boost::str(boost::format(mFormatString) % getCurrentLegIndex());
    if(zipOutput){
        ss << "_fullwave.bin";
    }
    else ss << "_fullwave.txt";
    fwf_detector->setOutputFilePath(
        mOutputFilePathString + getCurrentOutputPath(),
        ss.str(),
        getScanner()->isWriteWaveform()
    );

    // Trajectory writer
    if(mSurvey->scanner->trajectoryTimeInterval != 0.0){
        std::string trajectoryOutputPath = mOutputFilePathString +
                                           getTrajectoryOutputPath();
        mSurvey->scanner->setTrajectoryFileWriter(
            std::make_shared<SimpleSyncFileWriter>(trajectoryOutputPath)
            );
    }
}

void SurveyPlayback::clearPointcloudFile(){
    // Clear point cloud file for current leg
    string outputPath = this->outputPath + mOutputFilePathString +
                        getCurrentOutputPath();
    ostringstream s;s << "outputPath=" << outputPath << endl;
    logging::INFO(s.str());

    ofstream ofs;
    try {
        ofs.open(outputPath, ofstream::out | ofstream::trunc);
    }
    catch (std::exception &e) {
        logging::ERR(e.what());
    }
    ofs.close();
}


