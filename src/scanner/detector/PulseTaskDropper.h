#pragma once

#include <PulseThreadPoolInterface.h>
#include <noise/NoiseSource.h>
#include <noise/RandomnessGenerator.h>
#include <scanner/detector/PulseTask.h>
#include <scanner/detector/PulseThreadPool.h>
#include <util/threadpool/BuddingTaskDropper.h>
#if DATA_ANALYTICS >= 2
#include <dataanalytics/HDA_PulseRecorder.h>
using helios::analytics::HDA_PulseRecorder;
#endif

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a task dropper to deal with pulse tasks
 * @see TaskDropper
 * @see PulseThreadPool
 * @see PulseWarehouseThreadPool
 */
class PulseTaskDropper
  : public BuddingTaskDropper<PulseTaskDropper,
                              PulseTask,
                              PulseThreadPoolInterface,
                              std::vector<std::vector<double>>&,
                              RandomnessGenerator<double>&,
                              RandomnessGenerator<double>&,
                              NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                              ,
                              std::shared_ptr<HDA_PulseRecorder>
#endif
                              >
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for PulseTaskDropper
   * @param maxTasks Value to initialize maximum tasks limit
   * @see TaskDropper
   * @see TaskDropper::maxTasks
   */
  PulseTaskDropper(int const maxTasks = 32,
                   int const delta1 = 8,
                   int const initDelta1 = 8,
                   int const delta2 = 1,
                   char const lastSign = 0)
    : BuddingTaskDropper<PulseTaskDropper,
                         PulseTask,
                         PulseThreadPoolInterface,
                         std::vector<std::vector<double>>&,
                         RandomnessGenerator<double>&,
                         RandomnessGenerator<double>&,
                         NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                         ,
                         std::shared_ptr<HDA_PulseRecorder>
#endif
                         >(maxTasks, delta1, initDelta1, delta2, lastSign)
  {
  }
  virtual ~PulseTaskDropper() = default;

  // ***  TASK DROPPER METHODS *** //
  // ***************************** //
  using TaskDropper<PulseTask,
                    PulseThreadPoolInterface,
                    std::vector<std::vector<double>>&,
                    RandomnessGenerator<double>&,
                    RandomnessGenerator<double>&,
                    NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                    ,
                    std::shared_ptr<HDA_PulseRecorder>
#endif
                    >::drop; // To avoid overriding hides drop overloads
  using TaskDropper<PulseTask,
                    PulseThreadPoolInterface,
                    std::vector<std::vector<double>>&,
                    RandomnessGenerator<double>&,
                    RandomnessGenerator<double>&,
                    NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                    ,
                    std::shared_ptr<HDA_PulseRecorder>
#endif
                    >::tryDrop; // To avoid override hides tryDrop overloads
  /**
   * @brief Like TaskDropper::drop but dropping all pulse tasks through a
   *  pulse thread pool.
   * @param pool Pulse thread pool to be used for parallel execution
   * @see TaskDropper::drop
   * @see PulseThreadPool
   * @see PulseWarehouseThreadPool
   * @see FullWaveformPulseRunnable
   */
  inline void drop(PulseThreadPoolInterface& pool) override
  {
    pool.run_pulse_task(*this);
  }
  /**
   * @brief Like TaskDropper::tryDrop but dropping all pulse tasks through a
   *  pulse thread pool in a non-blocking way
   * @param pool Pulse thread pool to be used for parallel execution
   * @see TaskDropper::tryDrop
   */
  inline bool tryDrop(PulseThreadPoolInterface& pool) override
  {
    return pool.try_run_pulse_task(*this);
  }
};
