#include <scanner/BuddingScanningPulseProcess.h>
#include <scanner/Scanner.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
BuddingScanningPulseProcess::BuddingScanningPulseProcess(
  std::shared_ptr<Scanner> scanner,
  PulseTaskDropper& dropper,
  PulseThreadPool& pool,
  RandomnessGenerator<double>& randGen1,
  RandomnessGenerator<double>& randGen2,
  UniformNoiseSource<double>& intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
  ,
  std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
  )
  : ScanningPulseProcess(scanner)
  , dropper(dropper)
  , pool(pool)
  , randGen1(randGen1)
  , randGen2(randGen2)
  , intersectionHandlingNoiseSource(intersectionHandlingNoiseSource)
#if DATA_ANALYTICS >= 2
  , pulseRecorder(pulseRecorder)
#endif
{
  if (pool.getPoolSize() > 0) {
    if (pool.isDynamic()) { // Dynamic chunk schedule
      handler = [&](SimulatedPulse const& sp) -> void {
        handlePulseComputationParallelDynamic(sp);
      };
    } else { // Static chunk schedule
      handler = [&](SimulatedPulse const& sp) -> void {
        handlePulseComputationParallelStatic(sp);
      };
    }
  } else { // Sequential computation
    handler = [&](SimulatedPulse const& sp) -> void {
      handlePulseComputationSequential(sp);
    };
  }
#ifdef BUDDING_METRICS
  ofsBudding.open("budding_metrics.csv", std::ios_base::out);
  ofsBudding.setf(std::ios::fixed);
  ofsBudding.precision(6);
#endif
}

// ***  PULSE COMPUTATION  *** //
// *************************** //
void
BuddingScanningPulseProcess::onLegComplete()
{
  // If there is a pending chunk of tasks, sequentially compute it
  dropper.drop(apMatrix,
               randGen1,
               randGen2,
               intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
               ,
               pulseRecorder
#endif
  );
#ifdef BUDDING_METRICS
  ofsBudding.flush();
#endif
  pool.idleTimer.releaseStart();
  idleTimer.releaseStart();

  // Wait for threads to finish
  pool.join();
}
void
BuddingScanningPulseProcess::onSimulationFinished()
{
#ifdef BUDDING_METRICS
  ofsBudding.close();
#endif
}

// ***  INNER PULSE COMPUTATION  *** //
// ********************************* //
void
BuddingScanningPulseProcess::handlePulseComputationSequential(
  SimulatedPulse const& sp)
{
  // Sequential pulse computation
  shared_ptr<PulseTask> worker = ptf.build(*this, sp);
  (*worker)( // call functor
    apMatrix,
    randGen1,
    randGen2,
    intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
    ,
    pulseRecorder
#endif
  );
}
void
BuddingScanningPulseProcess::handlePulseComputationParallelDynamic(
  SimulatedPulse const& sp)
{
  // Submit pulse computation functor to thread pool
  char const status = dropper.tryAdd(pool, ptf.build(*this, sp));
  if (status == 1) { // Dropper successfully posted to thread pool
    if (idleTimer.hasStarted()) {
      long const idleNanos = idleTimer.getElapsedNanos();
      idleTimer.releaseStart();
#ifdef BUDDING_METRICS
      bool debugBuddingMetrics;
#endif
      dropper = dropper.evolve(
#ifdef BUDDING_METRICS
        debugBuddingMetrics,
#endif
        lastIdleNanos,
        idleNanos,
        idleTh,
        idleEps);
      lastIdleNanos = idleNanos;
      // Metric for debugging below (-DBUDDING_METRICS)
#ifdef BUDDING_METRICS
      if (debugBuddingMetrics) {
        ofsBudding << idleNanos << "," << dropper.getMaxTasks() << ","
                   << dropper.getDelta1() << "," << dropper.getDelta2() << ","
                   << (int)dropper.getLastSign() << "\n";
      }
#endif
    } else {
      dropper = dropper.emptyClone(); // No mutating budding
    }
  } else if (status == 2) { // Thread pool is full
    // Compute idle time (time between first idle and full work)
    if (pool.idleTimer.hasStarted()) {
      pool.idleTimer.stop();
      idleTimer.synchronize(pool.idleTimer);
      pool.idleTimer.releaseStart();
    }
    // Seq-do popped task
    (*dropper.popTask())(apMatrix,
                         randGen1,
                         randGen2,
                         intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
                         ,
                         pulseRecorder
#endif
    );
  }
}

void
BuddingScanningPulseProcess::handlePulseComputationParallelStatic(
  SimulatedPulse const& sp)
{
  // Submit pulse computation functor to thread pool
  char const status = dropper.tryAdd(pool, ptf.build(*this, sp));
  if (status == 1) { // Dropper successfully posted to thread pool
#ifdef BUDDING_METRICS
    if (idleTimer.hasStarted()) {
      long const idleNanos = idleTimer.getElapsedNanos();
      idleTimer.releaseStart();
      if (std::abs(idleNanos - lastIdleNanos) > idleEps) {
        lastIdleNanos = idleNanos;
        ofsBudding << idleNanos << "," << dropper.getMaxTasks() << ","
                   << dropper.getDelta1() << "," << dropper.getDelta2() << ","
                   << (int)dropper.getLastSign() << "\n";
      }
    }
#endif
    dropper = dropper.emptyClone(); // No mutating budding
  } else if (status == 2) {         // Thread pool is full
#ifdef BUDDING_METRICS
    // Compute idle time (time between first idle and full work)
    if (pool.idleTimer.hasStarted()) {
      pool.idleTimer.stop();
      idleTimer.synchronize(pool.idleTimer);
      pool.idleTimer.releaseStart();
    }
#endif
    // Seq-do popped task
    (*dropper.popTask())(apMatrix,
                         randGen1,
                         randGen2,
                         intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
                         ,
                         pulseRecorder
#endif
    );
  }
}
