#include <iostream>
#include "logging.hpp"

#include <chrono>
using namespace std::chrono;

#include "AbstractDetector.h"
#include <scanner/BuddingScanningPulseProcess.h>
#include <platform/InterpolatedMovingPlatformEgg.h>
#include <platform/InterpolatedMovingPlatform.h>
#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_StateJSONReporter.h>
using helios::analytics::HDA_StateJSONReporter;
#endif

#include "Simulation.h"
#include <TimeWatcher.h>
#include <DateTimeUtils.h>
#include <filems/facade/FMSFacade.h>


using namespace std;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Simulation::Simulation(
    int const parallelizationStrategy,
    std::shared_ptr<PulseThreadPoolInterface> pulseThreadPoolInterface,
    int chunkSize,
    std::string fixedGpsTimeStart
):
    parallelizationStrategy(parallelizationStrategy),
    threadPool(pulseThreadPoolInterface),
    taskDropper(chunkSize),
    stepLoop([&] () -> void{doSimStep();}),
    fixedGpsTimeStart(fixedGpsTimeStart),
    reporter(*this)

{
    currentGpsTime_ms = calcCurrentGpsTime();
}


// ***  SIMULATION METHODS  *** //
// **************************** //
void Simulation::prepareSimulation(int simFrequency_hz){
    // Mark as not finished
    finished = false;

    // Prepare platform to work with scanner
    this->mScanner->platform->prepareSimulation(
        this->mScanner->getPulseFreq_Hz()
    );

    // Prepare scanner
    this->mScanner->buildScanningPulseProcess(
        parallelizationStrategy,
        taskDropper,
        threadPool
    );

    // Prepare simulation
    setSimFrequency(this->mScanner->getPulseFreq_Hz());
    stepLoop.setCurrentStep(0);
}

void Simulation::doSimStep(){
	// Check for leg completion:
	if(
	    mScanner->scannerHead->rotateCompleted() &&
	    getScanner()->platform->waypointReached()
    ){
	    onLegComplete();
        return;
    }

    // Ordered execution of simulation components
	mScanner->platform->doSimStep(getScanner()->getPulseFreq_Hz());
    mScanner->doSimStep(mCurrentLegIndex, currentGpsTime_ms);
	mScanner->platform->scene->doSimStep();
    currentGpsTime_ms += 1000. / ((double)getScanner()->getPulseFreq_Hz());
    if (currentGpsTime_ms > 604800000.) currentGpsTime_ms -= 604800000.;
}


void Simulation::pause(bool pause) {
    if(pause == this->mPaused) return;
	this->mPaused = pause;

	if(pause){
        pauseLock = std::make_shared<std::unique_lock<std::mutex>>(
            std::unique_lock<std::mutex>(mutex)
        );
	}
	else{
        pauseLock = nullptr;
	}
}

void Simulation::shutdown(){
    finished = true;
    if(callback != nullptr && getCallbackFrequency() > 0){
        std::unique_lock<std::mutex> lock(*mScanner->cycleMeasurementsMutex);
        (*callback)(
            *mScanner->cycleMeasurements,
            *mScanner->cycleTrajectories,
            mScanner->detector->getFMS()->write
                .getMeasurementWriterOutputPath().string()
        );
    }
}


void Simulation::start() {
    // Report before starting simulation
    reporter.preStartReport();

    // Prepare to execute the main loop of simulation
    prepareSimulation(mScanner->getPulseFreq_Hz());
    size_t iter = 1;
    timeStart_ms = duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()
    ).count();
#ifdef DATA_ANALYTICS
    std::cout << "DATA ANALYTICS ..." << std::endl;
    HDA_StateJSONReporter sjr((SurveyPlayback *) this, "helios_state.json");
    sjr.report();
    std::cout << "DATA ANALYTICS!" << std::endl;
    std::exit(7);
#endif

    // Execute the main loop of the simulation
	while (!isStopped()) {
	    if(iter==1){ // TODO Pending : Does this lock make sense?
	        std::unique_lock<std::mutex> lock(mutex);
	    }

	    stepLoop.doStep();

		iter++;
		if(iter-1 == getCallbackFrequency()){ // TODO Pending : iter-1 by iter?
		    if(callback != nullptr){
                std::unique_lock<std::mutex> lock(
                    *mScanner->cycleMeasurementsMutex);
                (*callback)(
                    *mScanner->cycleMeasurements,
                    *mScanner->cycleTrajectories,
                    mScanner->detector->getFMS()->write
                        .getMeasurementWriterOutputPath().string()
                );
                mScanner->cycleMeasurements->clear();
                mScanner->cycleTrajectories->clear();
		    }
		    iter = 1;
		    condvar.notify_all();
		}
	}

	// Finish the main loop of the simulation
	long const timeMainLoopFinish = duration_cast<milliseconds>(
	    system_clock::now().time_since_epoch()
    ).count();
	double const seconds = ((double)(timeMainLoopFinish - timeStart_ms))
        / 1000.0;
	reporter.preFinishReport(seconds);
	mScanner->onSimulationFinished();

    // End of simulation report
    long const timeFinishAll = duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()
    ).count();
    double const secondsAll = ((double)(timeFinishAll - timeStart_ms))
                              / 1000.0;
    reporter.postFinishReport(secondsAll);

	// Shutdown the simulation (e.g. close all file output streams. Implemented in derived classes.)
	shutdown();
}


// ***  UTIL METHODS  *** //
// ********************** //
double Simulation::calcCurrentGpsTime(){
    long now;

    // Calc GPS time for fixed start time
    try{
        if(fixedGpsTimeStart != ""){
            if(fixedGpsTimeStart.find(":") != std::string::npos){
                // "YYYY-MM-DD hh:mm:ss"
                now = DateTimeUtils::dateTimeStrToMillis(fixedGpsTimeStart)
                      /1000000000L;
            }
            else{
                now = std::stol(fixedGpsTimeStart);
            }
        }
        else{
            // Calc GPS time for non fixed start time
            now = duration_cast<seconds>(
                system_clock::now().time_since_epoch()
            ).count();
        }
    }
    catch(std::exception &ex){
        std::stringstream ss;
        ss  << "Provided GPS start time was \"" << fixedGpsTimeStart << "\"\n"
            << "Please, ensure the format is either a POSIX timestamp, an "
            << "empty string \n"
            << "or a datetime with EXACT format: \"YYYY-MM-DD hh:mm:ss\" "
            << "(Don't forget the quotes)";
        logging::ERR(ss.str());
        throw ex;
    }

    // 315964809s is the difference between 1970-01-01 and 1980-01-06
    // 604800s per week -> resulting time is in ms since start of GPSweek
    return (double)((now - 315964809L) % 604800L) * 1000.;
}


// ***  GETTERs and SETTERs  *** //
// ***************************** //
void Simulation::setSimSpeedFactor(double factor) {
    if (factor <= 0) {
        factor = 0.0001;
    }

    if (factor > 10000) {
        factor = 10000;
    }

    this->mSimSpeedFactor = factor;

    stringstream ss;
    ss << "Simulation speed set to " << mSimSpeedFactor;
    logging::INFO(ss.str());
}

void Simulation::setScanner(shared_ptr<Scanner> scanner) {
    if (scanner == mScanner) {
        return;
    }

    logging::INFO("Simulation: Scanner changed!");

    this->mScanner = shared_ptr<Scanner>(scanner);
}
