#pragma once

#include <ResThreadPool.h>
#include <noise/RandomnessGenerator.h>
#include <UniformNoiseSource.h>

/**
 * @version 1.0
 * @brief Class implementing a thread pool to deal with pulse tasks
 * @see ResThreadPool
 */
class PulseThreadPool : public ResThreadPool<
    std::vector<std::vector<double>>&,
    RandomnessGenerator<double>&,
    RandomnessGenerator<double>&,
    NoiseSource<double>&
>{
protected:
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
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Thread pool constructor
     * @see thread_pool::pool_size
     * @param deviceAccuracy Parameter used to handle randomness generation
     *  impact on simulation results
     */
    explicit PulseThreadPool(
        std::size_t const _pool_size,
        double const deviceAccuracy
    ) :
        ResThreadPool<
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        >(_pool_size)
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

protected:
    /**
     * @brief Do a pulse task
     * @param task Pulse task
     * @see ThreadPool::do_task
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
};
