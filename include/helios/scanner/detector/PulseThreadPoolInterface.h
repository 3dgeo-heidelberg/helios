#pragma once

#include <helios/noise/NoiseSource.h>
#include <helios/noise/RandomnessGenerator.h>
#include <helios/scanner/detector/PulseTask.h>
#include <helios/util/threadpool/TaskDropper.h>
#if DATA_ANALYTICS >= 2
#include <helios/dataanalytics/HDA_PulseRecorder.h>
using helios::analytics::HDA_PulseRecorder;
#endif

#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface for thread pools supporting pulse tasks
 * @see PulseTaskDropper
 * @see PulseThreadPool
 * @see PulseWarehouseThreadPool
 */
class PulseThreadPoolInterface
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  virtual ~PulseThreadPoolInterface() = default;

  // *** PULSE THREAD POOL INTERFACE  *** //
  // ************************************ //
  /**
   * @brief Compute a chunk of pulse tasks in parallel, blocking fashion
   * @param dropper The pulse task dropper defining the chunk of pulse tasks
   *  to be computed
   * @see PulseTaskDropper
   */
  virtual void run_pulse_task(TaskDropper<PulseTask,
                                          PulseThreadPoolInterface,
                                          std::vector<std::vector<double>>&,
                                          RandomnessGenerator<double>&,
                                          RandomnessGenerator<double>&,
                                          NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                                          ,
                                          std::shared_ptr<HDA_PulseRecorder>
#endif
                                          >& dropper) = 0;
  /**
   * @brief Compute a chunk of pulse tasks in parallel, non-blocking fashion.
   *
   * Strictly speaking, it can be blocking. But the idea behind this method
   *  is that it will block only for thread-safe access purposes and not
   *  until running is guaranteed. So it might block, for instance, to post
   *  or submit a task to be computed, but not to run it.
   *
   * @param dropper The pulse task dropper defining the chunk of pulse tasks
   *  to be computed
   * @see PulseTaskDropper
   */
  virtual bool try_run_pulse_task(TaskDropper<PulseTask,
                                              PulseThreadPoolInterface,
                                              std::vector<std::vector<double>>&,
                                              RandomnessGenerator<double>&,
                                              RandomnessGenerator<double>&,
                                              NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                                              ,
                                              std::shared_ptr<HDA_PulseRecorder>
#endif
                                              >& dropper) = 0;
  /**
   * @brief Block until pending parallel executions have finished
   */
  virtual void join() = 0;

#if DATA_ANALYTICS >= 2
  /**
   * @brief Obtain the pointer to the pulse recorder being used by the
   *  pulse thread pool.
   * @return The pulse recorder used by the pulse thread pool.
   * @see helios::analytics::HDA_PulseRecorder
   */
  virtual std::shared_ptr<HDA_PulseRecorder> getPulseRecorder() = 0;
#endif
};
