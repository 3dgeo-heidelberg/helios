#pragma once
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

#include "Scanner.h"
#include "MeasurementsBuffer.h"
#include "Color4f.h"
#include <PulseTaskDropper.h>
#include <PulseThreadPool.h>
#include <FullWaveformPulseRunnable.h>
#include <SimulationCycleCallback.h>
#ifdef PYTHON_BINDING
#include <PySimulationCycleCallback.h>
#endif

/**
 * @brief Class representing a simulation
 */
class Simulation {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief How many nanoseconds there are in a second
     */
	static const long NANOSECONDS_PER_SECOND = 1000000000;

	/**
	 * @brief Number of threads available in the system
	 */
	unsigned numSysThreads = std::thread::hardware_concurrency(); //may return 0 when not able to detect

	/**
	 * @brief Pulse task dropper
	 * @see PulseTaskDropper
	 */
	PulseTaskDropper taskDropper;
	/**
	 * @brief Pulse thread pool
	 * @see PulseThreadPool
	 */
	PulseThreadPool threadPool;

	/**
	 * @brief Simulation speed factor
	 */
	double mSimSpeedFactor = 1;

	/**
	 * @brief Scanner used by the simulation
	 * @see Scanner
	 */
	std::shared_ptr<Scanner> mScanner = nullptr;

	/**
	 * @brief Simulation frequency. If it is 0 then no pause is possible.
	 */
	size_t simFrequency = 0;
	/**
	 * @brief Mutex to handle simulation pause and iterations on a
	 *  multi threading context
	 */
	std::mutex mutex;
	/**
	 * @brief Condition variable tu handle simulation iterations on a
	 *  multi threading context
	 */
	std::condition_variable condvar;
	/**
	 * @brief Shared pointer to the lock used to handle the mutex for
	 *  simulation pause purposes
	 * @see Simulation::mutex
	 */
	std::shared_ptr<std::unique_lock<std::mutex>> pauseLock = nullptr;

	/**
	 * @brief Flag specifying if the simulation has been stopped (true) or not
	 *  (false)
	 */
	bool mStopped = false;
	/**
	 * @brief Flag specifying if the simulation has been paused (true) or not
	 *  (false)
	 */
	bool mPaused = false;

    /**
     * @brief Time corresponding to simulation start (milliseconds)
     */
    long timeStart_ms = 0;

    /**
     * @brief Time corresponding to simulation start (currentGpsTime)
     */
    double currentGpsTime_ms = 0;





public:
    /**
     * @brief Index of leg at current simulation stage
     */
    unsigned int mCurrentLegIndex = 0;

    /**
     * @brief Flag specifying if simulation output must be exported to a file
     *  (true) or not (false)
     */
    bool exportToFile = true;
    /**
     * @brief Flag specifying if simulation must end when current leg has been
     *  completed (true) or not (false)
     */
	bool exitAtEnd = false;
	/**
	 * @brief Flag specifying if simulation has finished (true) or not (false)
	 */
    bool finished = false;

	std::shared_ptr<MeasurementsBuffer> mbuffer = nullptr;
    std::shared_ptr<SimulationCycleCallback> callback = nullptr;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simulation constructor
     * @param numThreads Number of threads to be used by the simulation
     * @param deviceAccuracy Parameter used to handle randomness generation
     *  impact on simulation results
     * @param parallelizationStrategy The parallelization strategy to be used
     * @param chunkSize The chunk size for the pulse task dropper
     * @param warehouseFactor The factor for the warehouse
     * @see PulseThreadPool
     * @see PulseTaskDropper
     * TODO Rethink : Add see for warehouse thread pool and task dropper
     */
    Simulation(
        unsigned const numThreads,
        double const deviceAccuracy,
        int const parallelizationStrategy=0,
        int const chunkSize=32,
        int const warehouseFactor=4
    );

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Perform computations for current simulation step
     */
	virtual void doSimStep();
	/**
	 * @brief Handle leg completion
	 */
	virtual void onLegComplete() = 0;

	/**
	 * @brief Start the simmulation
	 */
	void start();
	/**
	 * @brief Stop the simulation
	 *
	 * NOTICE this method will not stop simulation immediately but when
	 *  possible without compromising its execution. This means stop will
	 *  occur in a safe way.
	 */
	void stop() {this->mStopped = true;}
	/**
	 * @brief Pause or unpause the simulation
	 * @param pause  True to pause the simulation, false to unpause it
	 */
	void pause(bool pause);
	/**
	 * @brief Handle simulation shutdown
	 */
	void shutdown();

    /**
     * @brief Compute the current GPS time (milliseconds)
     * @return Current GPS time (milliseconds)
     */
    double calcCurrentGpsTime();


    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Set the simulation speed factor
     *
     * Factors less than or equal to 0 will be setted to 0.0001 and factors
     *  greater than 10000 will be truncated to 10000.
     *
     * In consequence, simulation speed factor will always be in interval
     *  \f$[0.0001, 10000]\f$ when setted through this setter
     *
     * @param factor New simulation speed factor
     * @see Simulation::mSimSpeedFactor
     */
	void setSimSpeedFactor(double factor);
    /**
     * @brief Obtain simulation speed factor
     * @return Simulation speed factor
     * @see Simulation::mSimSpeedFactor
     */
    double getSimSpeedFactor() {return this->mSimSpeedFactor;}

	/**
	 * @brief Set scanner for the simulation
	 * @param scanner New scanner for the simulation
	 * @see Simulation::mScanner
	 */
	void setScanner(std::shared_ptr<Scanner> scanner);
	/**
	 * @brief Obtain simulation scanner
	 * @return Simulation scanner
	 * @see Simulation::mScanner
	 */
	std::shared_ptr<Scanner> getScanner() {return this->mScanner;}


	/**
	 * @brief Check if simulation is paused (true) or not (false)
	 * @return True if simulation is paused, false otherwise
	 * @see Simulation::mPaused
	 */
	bool isPaused() {return this->mPaused;}
	/**
	 * @brief Check if simulation is stopped (true) or not (false)
	 * @return True if simulation is stopped, false otherwise
	 * @see Simulation::mStopped
	 */
	bool isStopped() {return this->mStopped;}
	/**
	 * @brief Obtain simulation frequency
	 * @return Simulation frequency (hertz)
	 * @see Simulation::simFrequency
	 */
	size_t getSimFrequency() {return simFrequency;}
	/**
	 * @brief Set simulation frequency
	 * @param simFrequency New simulation frequency (hertz)
	 * @see Simulation::simFrequency
	 */
	void setSimFrequency(size_t const simFrequency)
    {this->simFrequency = simFrequency;}
};
