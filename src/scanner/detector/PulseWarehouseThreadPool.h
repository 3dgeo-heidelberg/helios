#pragma once

#include <WarehouseThreadPool.h>
#include <PulseThreadPoolInterface.h>
#include <scanner/detector/PulseTaskDropper.h>
#include <noise/RandomnessGenerator.h>
#include <noise/UniformNoiseSource.h>

#include <memory>

using std::shared_ptr;
using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a warehouse thread pool to deal with pulse tasks
 * @see WarehouseThreadPool
 */
class PulseWarehouseThreadPool :
    public WarehouseThreadPool<PulseTaskDropper>,
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
	 * One per thread, to avoid spamming reallocations
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
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Pulse warehouse thread pool constructor
     * @see ThreadPool::pool_size
     * @param deviceAccuracy Parameter used to handle randomness generation
     *  impact on simulation results
     */
    explicit PulseWarehouseThreadPool(
        std::size_t const _pool_size,
        double const deviceAccuracy,
        std::size_t const maxTasks=256
    ) :
        WarehouseThreadPool<PulseTaskDropper>(_pool_size, maxTasks)
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

    virtual ~PulseWarehouseThreadPool(){
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
        throw HeliosException(
            "PulseWarehouseThreadPool::run_pulse_task is not supported.\n"
            "Try using try_run_pulse_task instead"
        );
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
        return post(make_shared<PulseTaskDropper>(
           static_cast<PulseTaskDropper &>(dropper)
        ));
    }
    /**
     * @see PulseThreadPoolInterface::join
     */
    inline void join() override{
        WarehouseThreadPool<PulseTaskDropper>::join();
    }

protected:
    // ***  WAREHOUSE THREADPOOL  *** //
    // ****************************** //
    /**
     * @brief Thread execute given task using its associated resources. It is,
     *  its apMatrix, its randomness generators and its noise source.
     * @see WarehouseThreadPool::doTask
     * @see PulseWarehouseThreadPool::apMatrices
     * @see PulseWarehouseThreadPool::randGens
     * @see PulseWarehouseThreadPool::randGens2
     * @see PulseWarehouseThreadPool::intersectionHandlingNoiseSources
     */
    void doTask(size_t const tid, shared_ptr<PulseTaskDropper> task) override {
        (*task)(
            apMatrices[tid],
            randGens[tid],
            randGens2[tid],
            intersectionHandlingNoiseSources[tid]
        );
    }
};