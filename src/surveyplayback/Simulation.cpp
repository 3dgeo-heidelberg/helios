#include <iostream>
#include "logging.hpp"

#include <chrono>
using namespace std::chrono;

#include "AbstractDetector.h"

#include "Simulation.h"
#include <TimeWatcher.h>

using namespace std;

Simulation::Simulation(unsigned numThreads, double deviceAccuracy):
threadPool(
    (numThreads == 0) ? numSysThreads : numThreads,
    deviceAccuracy
)
{
    mbuffer = make_shared<MeasurementsBuffer>();
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

	mScanner->platform->doSimStep(getScanner()->getPulseFreq_Hz());
	mScanner->doSimStep(threadPool, mCurrentLegIndex);
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
    if(callback != nullptr && simFrequency > 0){
        std::unique_lock<std::mutex> lock(*mScanner->cycleMeasurementsMutex);
        (*callback)(
            *mScanner->cycleMeasurements,
            *mScanner->cycleTrajectories
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
        this->mScanner->detector->mBuffer = this->mbuffer;
    }
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
    finished = false;

    // Prepare platform to work with scanner
    this->mScanner->platform->prepareSimulation(
        this->mScanner->getPulseFreq_Hz()
    );

    // Prepare simulation
	int stepCount = 0;
	timeStart_ms = duration_cast<milliseconds>(
	    system_clock::now().time_since_epoch()).count();


	// ############# BEGIN Main simulation loop ############
	size_t iter = 1;
	while (!isStopped()) {
	    if(iter==1){
	        std::unique_lock<std::mutex> lock(mutex);
	    }

		doSimStep();
		stepCount++;

		iter++;
		if(iter-1 == simFrequency){
		    if(callback != nullptr){
                std::unique_lock<std::mutex> lock(
                    *mScanner->cycleMeasurementsMutex);
                (*callback)(
                    *mScanner->cycleMeasurements,
                    *mScanner->cycleTrajectories
                );
                mScanner->cycleMeasurements->clear();
                mScanner->cycleTrajectories->clear();
		    }
		    iter = 1;
		    condvar.notify_all();
		}
	}

	long timeMainLoopFinish = duration_cast<milliseconds>(
	    system_clock::now().time_since_epoch()).count();
	long seconds = (timeMainLoopFinish - timeStart_ms) / 1000;

	stringstream ss;
	ss  << "stepCount = " << stepCount << "\n"
	    << "Main thread simulation loop finished in "<<seconds<<" sec."<<"\n"
	    << "Waiting for completion of pulse computation tasks...";
	logging::INFO(ss.str());
    ss.str("");

    // ########## BEGIN Loop that waits for the executor service to complete all tasks ###########
    long timeFinishAll = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	long secondsAll = (timeFinishAll - timeStart_ms) / 1000;
	ss << "Pulse computation tasks finished in " << secondsAll << " sec.";
	logging::INFO(ss.str());
	// ########## END Loop that waits for the executor service to complete all tasks ###########

	// Shutdown the simulation (e.g. close all file output streams. Implemented in derived classes.)
	shutdown();
}
