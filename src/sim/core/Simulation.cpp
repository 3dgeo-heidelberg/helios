#include <iostream>
#include "logging.hpp"

#include <chrono>
using namespace std::chrono;

#include "AbstractDetector.h"
#include <scanner/BuddingScanningPulseProcess.h>

#include "Simulation.h"
#include <TimeWatcher.h>
#include <DateTimeUtils.h>

using namespace std;


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
    fixedGpsTimeStart(fixedGpsTimeStart)
{
    mBuffer = make_shared<MeasurementsBuffer>();
    currentGpsTime_ms = calcCurrentGpsTime();
}

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
            mScanner->detector->outputFilePath.string()
        );
    }
}

void Simulation::setScanner(shared_ptr<Scanner> scanner) {
    if (scanner == mScanner) {
        return;
    }

    logging::INFO("Simulation: Scanner changed!");

    this->mScanner = shared_ptr<Scanner>(scanner);

    // Connect measurements buffer:
    if (this->mScanner != nullptr) {
        this->mScanner->detector->mBuffer = this->mBuffer;
    }
}


double Simulation::calcCurrentGpsTime(){
    long now;

    // Calc GPS time for fixed start time
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

    // 315964809s is the difference between 1970-01-01 and 1980-01-06
    // 604800s per week -> resulting time is in ms since start of GPSweek
    return (double)((now - 315964809L) % 604800L) * 1000.;
}


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

void Simulation::start() {
    // Prepare to execute the main loop of simulation
    prepareSimulation(mScanner->getPulseFreq_Hz());
    size_t iter = 1;
    timeStart_ms = duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()
    ).count();

    // Execute the main loop of the simulation
	while (!isStopped()) {
	    if(iter==1){
	        std::unique_lock<std::mutex> lock(mutex);
	    }

	    stepLoop.doStep();

		iter++;
		if(iter-1 == getCallbackFrequency()){
		    if(callback != nullptr){
                std::unique_lock<std::mutex> lock(
                    *mScanner->cycleMeasurementsMutex);
                (*callback)(
                    *mScanner->cycleMeasurements,
                    *mScanner->cycleTrajectories,
                    mScanner->detector->outputFilePath.string()
                );
                mScanner->cycleMeasurements->clear();
                mScanner->cycleTrajectories->clear();
		    }
		    iter = 1;
		    condvar.notify_all();
		}
	}

	// Finish the main loop of the simulation
	long timeMainLoopFinish = duration_cast<milliseconds>(
	    system_clock::now().time_since_epoch()).count();
	double seconds = ((double)(timeMainLoopFinish - timeStart_ms)) / 1000.0;
	mScanner->onSimulationFinished();

	// Report information about simulation main loop
	stringstream ss;
	ss  << "Elapsed simulation steps = " << stepLoop.getCurrentStep() << "\n"
	    << "Elapsed virtual time = " << stepLoop.getCurrentTime() << " sec.\n"
	    << "Main thread simulation loop finished in "<<seconds<<" sec."<<"\n"
	    << "Waiting for completion of pulse computation tasks...";
	logging::TIME(ss.str());
    ss.str("");
    long timeFinishAll = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	double secondsAll = ((double)(timeFinishAll - timeStart_ms)) / 1000.0;
	ss << "Pulse computation tasks finished in " << secondsAll << " sec.";
	logging::TIME(ss.str());

	// Shutdown the simulation (e.g. close all file output streams. Implemented in derived classes.)
	shutdown();
}
