#pragma once

#include <util/threadpool/TaskDropper.h>
#include <scanner/detector/PulseTask.h>
#include <scanner/detector/PulseThreadPool.h>
#include <noise/RandomnessGenerator.h>
#include <noise/NoiseSource.h>


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a task dropper to deal with pulse tasks
 * @see TaskDropper
 * @see PulseThreadPool
 */
class PulseTaskDropper : public TaskDropper<
    PulseTask,
    PulseThreadPool,
    std::vector<std::vector<double>>&,
    RandomnessGenerator<double>&,
    RandomnessGenerator<double>&,
    NoiseSource<double>&
>{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for PulseTaskDropper
     * @param maxTasks Value to initialize maximum tasks limit
     * @see TaskDropper
     * @see TaskDropper::maxTasks
     */
    PulseTaskDropper(size_t maxTasks=32) :
        TaskDropper<
            PulseTask,
            PulseThreadPool,
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        >(maxTasks)
    {}
    virtual ~PulseTaskDropper() = default;

    // ***  TASK DROPPER METHODS *** //
    // ***************************** //
    /**
     * @brief Like TaskDropper::drop but dropping all pulse tasks through a
     *  pulse thread pool
     * @brief Callback for pulse tasks dropper called through pulse thread
     *  pool
     * @param pool Pulse thread pool to be used for parallel execution
     * @see TaskDropper::drop
     * @see PulseThreadPool
     * @see FullWaveformPulseRunnable
     */
    void drop(PulseThreadPool &pool) override {pool.run_res_task(*this);}
};
