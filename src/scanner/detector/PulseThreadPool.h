#pragma once

#include <ResThreadPool.h>
#include <scanner/detector/PulseThreadPoolInterface.h>
#include <noise/RandomnessGenerator.h>
#include <noise/UniformNoiseSource.h>
#include <TimeWatcher.h>

/**
 * @version 1.0
 * @brief Class implementing a thread pool to deal with pulse tasks
 * @see ResThreadPool
 */
class PulseThreadPool :
    public ResThreadPool<
        std::vector<std::vector<double>>&,
        RandomnessGenerator<double>&,
        RandomnessGenerator<double>&,
        NoiseSource<double>&
    >,
    public PulseThreadPoolInterface
{
#ifdef DATA_ANALYTICS
public:
#else
protected:
#endif
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
	 * Alpha prime matrices for MarquardtFitter.
	 * One per possible thread, to avoid spamming reallocations
	 */
    std::vector<std::vector<double>> *apMatrices;

    /**
     * @brief First randomness generators (general purpose), one per thread
     */
    RandomnessGenerator<double> *randGens; // General purpose
    /**
	 * @brief Second randomness generators (to substitute old box muller), one
	 *  per thread
	 */
    RandomnessGenerator<double> *randGens2; // To substitute BoxMuller
    /**
	 * @brief Intersection handling noise sources, one per thread
	 */
    UniformNoiseSource<double> *intersectionHandlingNoiseSources;

public:
    /**
	 * @brief Time watcher to count the amount of idle time.
     *
     * It is started when all threads are occupied as soon as the first thread
     *  becomes available (finishes its job). It must stopped by the
     *  asynchronous process at the correct point. This means the idle timer
     *  is meant to be used with non-blocking task posting, it is when using
     *  ResThreadPool::try_run_res_task method instead of
     *  ResThreadPool::run_res_task
     *
     * @see ResThreadPool::try_run_res_task
	 */
    TimeWatcher idleTimer;
    /**
     * @brief Specify whether the pulse thread pool uses a dynamic chunk size
     *  strategy (true) or a static one (false)
     */
    bool dynamic;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Pulse thread pool constructor
     * @see ThreadPool::pool_size
     * @param deviceAccuracy Parameter used to handle randomness generation
     *  impact on simulation results
     */
    explicit PulseThreadPool(
        std::size_t const _pool_size,
        double const deviceAccuracy,
        bool const dynamic
    ) :
        ResThreadPool<
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        >(_pool_size),
        dynamic(dynamic)
    {
        // Allocate
        apMatrices = new std::vector<std::vector<double>>[this->pool_size];
        randGens = new RandomnessGenerator<double>[this->pool_size];
        randGens2 = new RandomnessGenerator<double>[this->pool_size];
        intersectionHandlingNoiseSources =
            new UniformNoiseSource<double>[this->pool_size];

        // Initialize
        for (std::size_t i = 0; i < this->pool_size; ++i){
            randGens[i] = *DEFAULT_RG;
            randGens[i].computeUniformRealDistribution(0.0, 1.0);
            randGens[i].computeNormalDistribution(0.0, deviceAccuracy);
            randGens2[i] = *DEFAULT_RG;
            randGens2[i].computeNormalDistribution(0.0, 1.0);
            intersectionHandlingNoiseSources[i] = UniformNoiseSource<double>(
                *DEFAULT_RG, 0.0, 1.0
            );
        }
    }

    virtual ~PulseThreadPool(){
        // Release memory
        delete[] apMatrices;
        delete[] randGens;
        delete[] randGens2;
        delete[] intersectionHandlingNoiseSources;
    }

    // *** PULSE THREAD POOL INTERFACE  *** //
    // ************************************ //
    /**
     * @see PulseThreadPoolInterface::run_pulse_task
     */
    inline void run_pulse_task(
        TaskDropper<
            PulseTask,
            PulseThreadPoolInterface,
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        > &dropper
    ) override {
        run_res_task(dropper);
    }
    /**
     * @see PulseThreadPoolInterface::try_run_pulse_task
     */
    inline bool try_run_pulse_task(
        TaskDropper<
            PulseTask,
            PulseThreadPoolInterface,
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        > &dropper
    ) override {
        return try_run_res_task(dropper);
    }
    /**
     * @see PulseThreadPoolInterface::join
     */
    inline void join() override{
        ResThreadPool<
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        >::join();
    }

protected:
    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Do a pulse task
     * @param task Pulse task
     * @see ResThreadPool::do_task
     */
    inline void do_res_task(
        boost::function<void(
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        )> &task,
        int const resourceIdx
    ) override{
        task(
            apMatrices[resourceIdx],
            randGens[resourceIdx],
            randGens2[resourceIdx],
            intersectionHandlingNoiseSources[resourceIdx]
        );
    }
    /**
	 * @brief Override task wrapping so when the full thread group is used
     *  the time at which first occupied thread becomes available is registered
     * @see PulseThreadPool::firstAvailableTime
	 */
    void wrap_res_task(
        boost::function<void(
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        )> &task,
        int const resourceIdx
    ) override {
        // Run the user supplied task.
        try{
            do_res_task(task, resourceIdx);
        }
        // Suppress all exceptions.
        catch (const std::exception &e) {
            std::stringstream ss;
            ss << "PulseThreadPool::wrap_res_task EXCEPTION: " << e.what();
            logging::WARN(ss.str());
        }

        // Task has finished, so increment count of available threads.
        boost::unique_lock<boost::mutex> lock(this->mutex_);
        ++(this->available_);
        resourceSetAvailable[resourceIdx] = true;
        idleTimer.startIfNull(); // Start time at first idle thread
        lock.unlock();
        this->cond_.notify_one();
    }

public:
    /**
     * @brief Check if the pulse thread pool is operating on dynamic mode
     *  (true) or not (false)
     * @return True if dynamic chunk scheduling, false if static
     */
    virtual inline bool isDynamic() const {return dynamic;}
};
