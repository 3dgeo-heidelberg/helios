#include <helios/util/logger/logging.hpp>
#include <iostream>

#include <chrono>
using namespace std::chrono;

#include <helios/platform/InterpolatedMovingPlatform.h>
#include <helios/platform/InterpolatedMovingPlatformEgg.h>
#include <helios/scanner/BuddingScanningPulseProcess.h>
#include <helios/scanner/detector/AbstractDetector.h>
#include <helios/scene/dynamic/DynScene.h>
#ifdef DATA_ANALYTICS
#include <helios/dataanalytics/HDA_SimStepRecorder.h>
using helios::analytics::HDA_Recorder;
using helios::analytics::HDA_SimStepRecorder;
using helios::analytics::HDA_StateJSONReporter;
#endif

#include <helios/filems/facade/FMSFacade.h>
#include <helios/sim/core/Simulation.h>
#include <helios/util/DateTimeUtils.h>
#include <helios/util/TimeWatcher.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Simulation::Simulation(
  int const parallelizationStrategy,
  std::shared_ptr<PulseThreadPoolInterface> pulseThreadPoolInterface,
  int chunkSize,
  std::string fixedGpsTimeStart,
  bool const legacyEnergyModel)
  : parallelizationStrategy(parallelizationStrategy)
  , threadPool(pulseThreadPoolInterface)
  , taskDropper(chunkSize)
  , stepLoop([&]() -> void { doSimStep(); })
  , fixedGpsTimeStart(fixedGpsTimeStart)
  , legacyEnergyModel(legacyEnergyModel)
  , reporter(*this)
{
  currentGpsTime_ns = calcCurrentGpsTime();
}

// ***  SIMULATION METHODS  *** //
// **************************** //
void
Simulation::prepareSimulation(int simFrequency_hz)
{
  // Mark as not finished
  finished = false;

  // Prepare platform to work with scanner
  this->mScanner->platform->prepareSimulation(
    this->mScanner->getPulseFreq_Hz());

  // Prepare scanner
  this->mScanner->prepareSimulation(legacyEnergyModel);
  this->mScanner->buildScanningPulseProcess(
    parallelizationStrategy, taskDropper, threadPool);

  // Prepare simulation
  setSimFrequency(this->mScanner->getPulseFreq_Hz());
  stepLoop.setCurrentStep(0);
  stepGpsTime_ns = 1000000000. * stepLoop.getPeriod();

  // Prepare scene (mostly for dynamic scenes)
  mScanner->platform->scene->prepareSimulation(simFrequency_hz);
}

void
Simulation::doSimStep()
{
  // Check for leg completion:
  if (mScanner->getScannerHead(0)->rotateCompleted() &&
      mScanner->platform->waypointReached()) {
    onLegComplete();
    return;
  }

  // Ordered execution of simulation components
  mScanner->platform->doSimStep(mScanner->getPulseFreq_Hz());
  mScanner->doSimStep(mCurrentLegIndex, currentGpsTime_ns);
  mScanner->platform->scene->doSimStep();
  currentGpsTime_ns += stepGpsTime_ns;
  if (currentGpsTime_ns > 604800000000000.)
    currentGpsTime_ns -= 604800000000000.;
}

void
Simulation::pause(bool pause)
{
  if (pause == this->mPaused)
    return;
  this->mPaused = pause;

  if (pause) {
    pauseLock = std::make_shared<std::unique_lock<std::mutex>>(
      std::unique_lock<std::mutex>(mutex));
  } else {
    pauseLock = nullptr;
  }
}

void
Simulation::shutdown()
{
  finished = true;
  if (callback != nullptr && getCallbackFrequency() > 0) {
    std::string const mwOutPath =
      (exportToFile)
        ? mScanner->fms->write.getMeasurementWriterOutputPath().string()
        : "";
    std::unique_lock<std::mutex> lock(*mScanner->cycleMeasurementsMutex);
    (*callback)(
      *mScanner->cycleMeasurements, *mScanner->cycleTrajectories, mwOutPath);
  }
}

void
Simulation::start()
{
  // Report before starting simulation
  reporter.preStartReport();

  // Prepare to execute the main loop of simulation
  prepareSimulation(mScanner->getPulseFreq_Hz());
  timeStart_ns =
    duration_cast<nanoseconds>(system_clock::now().time_since_epoch());

#ifdef DATA_ANALYTICS
  HDA_StateJSONReporter sjr((SurveyPlayback*)this, "helios_state.json");
  sjr.report();
  HDA_SimStepRecorder ssr((SurveyPlayback*)this, "helios_sim_records");
#endif

  // Play simulation
  simPlayer = std::make_unique<SimulationPlayer>(*this);
  int simLoopIndex = 0;
  std::stringstream ss;
  while (simPlayer->hasPendingPlays()) {
    ss.str("");
    ss << "Starting simulation loop " << simLoopIndex + 1 << " ...";
    logging::INFO(ss.str());
    doSimLoop(
#ifdef DATA_ANALYTICS
      ssr
#endif
    );
    // NOTE there is no need for a sync. barrier after the last iteration
    // because end of simulation will handle it.
    ss.str("");
    ss << "Finishing simulation loop " << simLoopIndex + 1 << " ...";
    logging::INFO(ss.str());
    simPlayer->endPlay();
    ss.str("");
    ss << "Finished simulation loop " << simLoopIndex + 1 << ".";
    logging::INFO(ss.str());
    ++simLoopIndex;
  }
  simPlayer = nullptr;

#ifdef DATA_ANALYTICS
  // Finish data analytics stuff
  ssr.delayedRecord();
  ssr.closeBuffers();
#endif

  // Finish the main loop of the simulation
  std::chrono::nanoseconds timeMainLoopFinish =
    duration_cast<std::chrono::nanoseconds>(
      system_clock::now().time_since_epoch());
  double const seconds =
    ((double)(timeMainLoopFinish - timeStart_ns).count()) / 1000000000.0;
  reporter.preFinishReport(seconds);
  mScanner->onSimulationFinished();

  // End of simulation report
  std::chrono::nanoseconds const timeFinishAll =
    duration_cast<std::chrono::nanoseconds>(
      system_clock::now().time_since_epoch());
  double const secondsAll =
    ((double)(timeFinishAll - timeStart_ns).count()) / 1000000000.0;
  reporter.postFinishReport(secondsAll);

  // Shutdown the simulation (e.g. close all file output streams. Implemented in
  // derived classes.)
  shutdown();
}

void
Simulation::doSimLoop(
#ifdef DATA_ANALYTICS
  HDA_Recorder& _ssr
#endif
)
{
#ifdef DATA_ANALYTICS
  HDA_SimStepRecorder& ssr = static_cast<HDA_SimStepRecorder&>(_ssr);
#endif
  size_t iter = 1;
  // Execute the main loop of the simulation
  while (!isStopped()) {
    if (iter == 1) { // TODO Pending : Does this lock make sense?
      std::unique_lock<std::mutex> lock(mutex);
    }

    stepLoop.doStep();

    iter++;
    if (iter - 1 == getCallbackFrequency()) { // TODO Pending : iter-1 by iter?
      if (callback != nullptr) {
        std::string const mwOutPath =
          (exportToFile)
            ? mScanner->fms->write.getMeasurementWriterOutputPath().string()
            : "";
        std::unique_lock<std::mutex> lock(*mScanner->cycleMeasurementsMutex);
        (*callback)(*mScanner->cycleMeasurements,
                    *mScanner->cycleTrajectories,
                    mwOutPath);
        mScanner->cycleMeasurements->clear();
        mScanner->cycleTrajectories->clear();
      }
      iter = 1;
      condvar.notify_all();
    }

#ifdef DATA_ANALYTICS
    ssr.record();
#endif
  }
}

// ***  UTIL METHODS  *** //
// ********************** //
double
Simulation::calcCurrentGpsTime()
{
  long now;

  // Calc GPS time for fixed start time
  try {
    if (fixedGpsTimeStart != "") {
      if (fixedGpsTimeStart.find(":") != std::string::npos) {
        // "YYYY-MM-DD hh:mm:ss"
        now = DateTimeUtils::dateTimeStrToSeconds(fixedGpsTimeStart);
      } else {
        now = std::stol(fixedGpsTimeStart);
      }
    } else {
      // Calc GPS time for non fixed start time
      now =
        duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
    }
  } catch (std::exception& ex) {
    std::stringstream ss;
    ss << "Provided GPS start time was \"" << fixedGpsTimeStart << "\"\n"
       << "Please, ensure the format is either a POSIX timestamp, an "
       << "empty string \n"
       << "or a datetime with EXACT format: \"YYYY-MM-DD hh:mm:ss\" "
       << "(Don't forget the quotes)";
    logging::ERR(ss.str());
    throw ex;
  }

  // 315964809s is the difference between 1970-01-01 and 1980-01-06
  // 604800s per week -> resulting time is in ns since start of GPSweek
  return (double)((now - 315964809L) % 604800L) * 1000000000.;
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void
Simulation::setSimSpeedFactor(double factor)
{
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

void
Simulation::setScanner(std::shared_ptr<Scanner> scanner)
{
  if (scanner == mScanner) {
    return;
  }

  logging::INFO("Simulation: Scanner changed!");

  this->mScanner = std::shared_ptr<Scanner>(scanner);
}
