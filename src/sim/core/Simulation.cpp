#include "logging.hpp"
#include <iostream>

#include <chrono>
using namespace std::chrono;

#include "AbstractDetector.h"
#include <platform/InterpolatedMovingPlatform.h>
#include <platform/InterpolatedMovingPlatformEgg.h>
#include <scanner/BuddingScanningPulseProcess.h>
#include <scene/dynamic/DynScene.h>
#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_SimStepRecorder.h>
using helios::analytics::HDA_Recorder;
using helios::analytics::HDA_SimStepRecorder;
using helios::analytics::HDA_StateJSONReporter;
#endif

#include "Simulation.h"
#include <DateTimeUtils.h>
#include <TimeWatcher.h>
#include <filems/facade/FMSFacade.h>

#include <algorithm>
#include <stdexcept>

namespace {
std::string
resolveMeasurementOutputPath(const std::shared_ptr<Scanner>& scanner,
                             bool const exportToFile)
{
  if (!exportToFile || scanner == nullptr || scanner->fms == nullptr) {
    return "";
  }
  return scanner->fms->write.getMeasurementWriterOutputPath().string();
}
}

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
Simulation::addHook(SurveyHookRegistration reg)
{
  hookRegistrations.push_back(std::move(reg));
}

void
Simulation::clearHooks()
{
  hookRegistrations.clear();
}

void
Simulation::initializeHooksForRun()
{
  hookStepIndex = 0;
  hookPlayIndex = 0;
  hookFailed.store(false);

  {
    std::lock_guard<std::mutex> lock(hookExceptionMutex);
    hookException = nullptr;
  }

  for (auto& reg : hookRegistrations) {
    reg.firedOnce = false;
    reg.nextFire_s = reg.simTime_s;
    reg.lastMeasurementIndex = 0;
    reg.lastTrajectoryIndex = 0;
  }
}

void
Simulation::emitLegStartHook()
{
  if (hookRegistrations.empty()) {
    return;
  }

  hookPlayIndex = (simPlayer != nullptr)
                    ? static_cast<uint32_t>(simPlayer->getNumComputedPlays())
                    : 0;

  double const simTime_s = stepLoop.getCurrentTime();
  for (auto& reg : hookRegistrations) {
    if (reg.point != HookPoint::LEG_START) {
      continue;
    }

    HookContext ctx;
    ctx.point = HookPoint::LEG_START;
    ctx.stepIndex = hookStepIndex;
    ctx.playIndex = hookPlayIndex;
    ctx.legIndex = static_cast<int>(mCurrentLegIndex);
    ctx.simTime_s = simTime_s;
    ctx.gpsTime_s = currentGpsTime_ns * 1e-9;
    ctx.outputPath = resolveMeasurementOutputPath(mScanner, exportToFile);
    ctx.scheduledTime_s = simTime_s;
    ctx.lag_s = 0.0;
    enrichHookContext(ctx);

    dispatchHook(reg, ctx);
    if (hasHookFailure()) {
      return;
    }
  }
}

void
Simulation::emitLegEndHook()
{
  if (hookRegistrations.empty()) {
    return;
  }

  hookPlayIndex = (simPlayer != nullptr)
                    ? static_cast<uint32_t>(simPlayer->getNumComputedPlays())
                    : 0;

  double const simTime_s = stepLoop.getCurrentTime();
  for (auto& reg : hookRegistrations) {
    if (reg.point != HookPoint::LEG_END) {
      continue;
    }

    HookContext ctx;
    ctx.point = HookPoint::LEG_END;
    ctx.stepIndex = hookStepIndex;
    ctx.playIndex = hookPlayIndex;
    ctx.legIndex = static_cast<int>(mCurrentLegIndex);
    ctx.simTime_s = simTime_s;
    ctx.gpsTime_s = currentGpsTime_ns * 1e-9;
    ctx.outputPath = resolveMeasurementOutputPath(mScanner, exportToFile);
    ctx.scheduledTime_s = simTime_s;
    ctx.lag_s = 0.0;
    enrichHookContext(ctx);

    dispatchHook(reg, ctx);
    if (hasHookFailure()) {
      return;
    }
  }
}

void
Simulation::emitTimedHooks(double simTime_s)
{
  if (hookRegistrations.empty()) {
    return;
  }

  hookPlayIndex = (simPlayer != nullptr)
                    ? static_cast<uint32_t>(simPlayer->getNumComputedPlays())
                    : 0;

  for (auto& reg : hookRegistrations) {
    if (reg.point == HookPoint::SIM_TIME_ONCE) {
      if (reg.firedOnce || simTime_s < reg.simTime_s) {
        continue;
      }

      HookContext ctx;
      ctx.point = HookPoint::SIM_TIME_ONCE;
      ctx.stepIndex = hookStepIndex;
      ctx.playIndex = hookPlayIndex;
      ctx.legIndex = static_cast<int>(mCurrentLegIndex);
      ctx.simTime_s = simTime_s;
      ctx.gpsTime_s = currentGpsTime_ns * 1e-9;
      ctx.outputPath = resolveMeasurementOutputPath(mScanner, exportToFile);
      ctx.scheduledTime_s = reg.simTime_s;
      ctx.lag_s = simTime_s - reg.simTime_s;
      enrichHookContext(ctx);

      dispatchHook(reg, ctx);
      if (hasHookFailure()) {
        return;
      }
      reg.firedOnce = true;
      continue;
    }

    if (reg.point == HookPoint::SIM_TIME_PERIODIC) {
      if (reg.period_s <= 0.0 || simTime_s < reg.nextFire_s) {
        continue;
      }

      double const scheduledTime_s = reg.nextFire_s;

      HookContext ctx;
      ctx.point = HookPoint::SIM_TIME_PERIODIC;
      ctx.stepIndex = hookStepIndex;
      ctx.playIndex = hookPlayIndex;
      ctx.legIndex = static_cast<int>(mCurrentLegIndex);
      ctx.simTime_s = simTime_s;
      ctx.gpsTime_s = currentGpsTime_ns * 1e-9;
      ctx.outputPath = resolveMeasurementOutputPath(mScanner, exportToFile);
      ctx.scheduledTime_s = scheduledTime_s;
      ctx.lag_s = simTime_s - scheduledTime_s;
      enrichHookContext(ctx);

      dispatchHook(reg, ctx);
      if (hasHookFailure()) {
        return;
      }
      reg.nextFire_s += reg.period_s;
    }
  }
}

void
Simulation::emitPeriodicTailHooks(double simTime_s)
{
  if (hookRegistrations.empty()) {
    return;
  }

  hookPlayIndex = (simPlayer != nullptr)
                    ? static_cast<uint32_t>(simPlayer->getNumComputedPlays())
                    : 0;

  for (auto& reg : hookRegistrations) {
    if (reg.point != HookPoint::SIM_TIME_PERIODIC ||
        reg.endOfLegPolicy == HookEndOfLegPolicy::NONE) {
      continue;
    }

    bool shouldFlush = true;
    if (reg.payload == HookPayload::SINCE_LAST) {
      shouldFlush = true;
      if (mScanner != nullptr && mScanner->allMeasurements != nullptr &&
          mScanner->allTrajectories != nullptr &&
          mScanner->allMeasurementsMutex != nullptr) {
        std::unique_lock<std::mutex> lock(*mScanner->allMeasurementsMutex);
        shouldFlush =
          reg.lastMeasurementIndex < mScanner->allMeasurements->size() ||
          reg.lastTrajectoryIndex < mScanner->allTrajectories->size();
      }
    }

    if (shouldFlush) {
      HookContext ctx;
      ctx.point = HookPoint::SIM_TIME_PERIODIC;
      ctx.stepIndex = hookStepIndex;
      ctx.playIndex = hookPlayIndex;
      ctx.legIndex = static_cast<int>(mCurrentLegIndex);
      ctx.simTime_s = simTime_s;
      ctx.gpsTime_s = currentGpsTime_ns * 1e-9;
      ctx.outputPath = resolveMeasurementOutputPath(mScanner, exportToFile);
      ctx.scheduledTime_s = simTime_s;
      ctx.lag_s = 0.0;
      enrichHookContext(ctx);

      dispatchHook(reg, ctx);
      if (hasHookFailure()) {
        return;
      }
    }

    if (reg.endOfLegPolicy == HookEndOfLegPolicy::FLUSH_AND_RESET &&
        reg.period_s > 0.0) {
      reg.nextFire_s = simTime_s + reg.period_s;
    }
  }
}

void
Simulation::enrichHookContext(HookContext& ctx) const
{
  (void)ctx;
}

void
Simulation::dispatchHook(SurveyHookRegistration& reg, HookContext const& ctx)
{
  if (hasHookFailure() || !reg.fn) {
    return;
  }

  if (reg.barrier && mScanner != nullptr) {
    mScanner->flushPendingPulseTasks();
  }

  HookContext localCtx = ctx;
  std::vector<Measurement> pointsPayload;
  std::vector<Trajectory> trajectoriesPayload;
  std::vector<Measurement> const* pointsPayloadPtr = nullptr;
  std::vector<Trajectory> const* trajectoriesPayloadPtr = nullptr;
  size_t nextMeasurementIndex = reg.lastMeasurementIndex;
  size_t nextTrajectoryIndex = reg.lastTrajectoryIndex;

  bool const needsPayload = reg.payload != HookPayload::METADATA_ONLY;
  bool const canSnapshot = mScanner != nullptr &&
                           mScanner->allMeasurements != nullptr &&
                           mScanner->allTrajectories != nullptr &&
                           mScanner->allMeasurementsMutex != nullptr;

  if (needsPayload && !canSnapshot) {
    {
      std::lock_guard<std::mutex> lock(hookExceptionMutex);
      if (!hookException) {
        hookException = std::make_exception_ptr(
          std::runtime_error("Hook payload requested but scanner tracking "
                             "buffers are unavailable."));
      }
    }
    hookFailed.store(true);
    stop();
    return;
  }

  if (canSnapshot) {
    std::unique_lock<std::mutex> lock(*mScanner->allMeasurementsMutex);

    size_t const totalPoints = mScanner->allMeasurements->size();
    size_t const totalTrajectories = mScanner->allTrajectories->size();

    localCtx.totalPoints = totalPoints;
    localCtx.totalTrajectories = totalTrajectories;

    if (reg.payload == HookPayload::SINCE_LAST) {
      size_t const firstPoint = std::min(reg.lastMeasurementIndex, totalPoints);
      size_t const firstTrajectory =
        std::min(reg.lastTrajectoryIndex, totalTrajectories);

      pointsPayload.assign(mScanner->allMeasurements->cbegin() + firstPoint,
                           mScanner->allMeasurements->cend());
      trajectoriesPayload.assign(mScanner->allTrajectories->cbegin() +
                                   firstTrajectory,
                                 mScanner->allTrajectories->cend());

      nextMeasurementIndex = totalPoints;
      nextTrajectoryIndex = totalTrajectories;
      pointsPayloadPtr = &pointsPayload;
      trajectoriesPayloadPtr = &trajectoriesPayload;
    } else if (reg.payload == HookPayload::ALL_POINTS) {
      pointsPayload.assign(mScanner->allMeasurements->cbegin(),
                           mScanner->allMeasurements->cend());
      trajectoriesPayload.assign(mScanner->allTrajectories->cbegin(),
                                 mScanner->allTrajectories->cend());

      pointsPayloadPtr = &pointsPayload;
      trajectoriesPayloadPtr = &trajectoriesPayload;
    }
  }

  localCtx.payloadPoints =
    pointsPayloadPtr != nullptr ? pointsPayload.size() : 0;
  localCtx.payloadTrajectories =
    trajectoriesPayloadPtr != nullptr ? trajectoriesPayload.size() : 0;

  try {
    reg.fn(localCtx, pointsPayloadPtr, trajectoriesPayloadPtr);
    if (reg.payload == HookPayload::SINCE_LAST) {
      reg.lastMeasurementIndex = nextMeasurementIndex;
      reg.lastTrajectoryIndex = nextTrajectoryIndex;
    }
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(hookExceptionMutex);
      if (!hookException) {
        hookException = std::current_exception();
      }
    }
    hookFailed.store(true);
    stop();
  }
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
}

void
Simulation::start()
{
  // Report before starting simulation
  reporter.preStartReport();

  // Prepare to execute the main loop of simulation
  prepareSimulation(mScanner->getPulseFreq_Hz());
  initializeHooksForRun();
  timeStart_ns =
    duration_cast<nanoseconds>(system_clock::now().time_since_epoch());

#ifdef DATA_ANALYTICS
  HDA_StateJSONReporter sjr((SurveyPlayback*)this, "helios_state.json");
  sjr.report();
  HDA_SimStepRecorder ssr((SurveyPlayback*)this, "helios_sim_records");
#endif

  // Play simulation
  simPlayer = std::make_unique<SimulationPlayer>(*this);
  if (simPlayer->hasPendingPlays()) {
    emitLegStartHook();
  }
  int simLoopIndex = 0;
  std::stringstream ss;
  while (simPlayer->hasPendingPlays()) {
    hookPlayIndex = static_cast<uint32_t>(simPlayer->getNumComputedPlays());
    hookStepIndex = 0;
    for (auto& reg : hookRegistrations) {
      if (reg.point == HookPoint::SIM_TIME_ONCE ||
          reg.point == HookPoint::SIM_TIME_PERIODIC) {
        reg.firedOnce = false;
        reg.nextFire_s = reg.simTime_s;
      }
    }

    ss.str("");
    ss << "Starting simulation loop " << simLoopIndex + 1 << " ...";
    logging::INFO(ss.str());
    doSimLoop(
#ifdef DATA_ANALYTICS
      ssr
#endif
    );

    if (hasHookFailure()) {
      break;
    }

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

  if (hasHookFailure()) {
    std::exception_ptr ex;
    {
      std::lock_guard<std::mutex> lock(hookExceptionMutex);
      ex = hookException;
    }
    if (ex) {
      std::rethrow_exception(ex);
    }
    throw std::runtime_error("Simulation hook callback failed.");
  }
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
  // Execute the main loop of the simulation
  while (!isStopped()) {
    stepLoop.doStep();
    if (hasHookFailure()) {
      break;
    }

    ++hookStepIndex;
    emitTimedHooks(stepLoop.getCurrentTime());
    if (hasHookFailure()) {
      break;
    }

    condvar.notify_all();

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
