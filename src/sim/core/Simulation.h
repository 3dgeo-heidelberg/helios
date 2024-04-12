#pragma once

#include <Scanner.h>
#include <Color4f.h>
#include <SimulationStepLoop.h>
#include <PulseTaskDropper.h>
#include <PulseThreadPoolInterface.h>
#include <FullWaveformPulseRunnable.h>
#include <SimulationCycleCallback.h>
#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_StateJSONReporter.h>
#endif
#include <SimulationReporter.h>
#include <SimulationPlayer.h>

#include <chrono>

/**
 * @brief Class representing a simulation
 */
class Simulation {
private:
    friend class SimulationReporter;
    friend class SimulationPlayer;
#ifdef DATA_ANALYTICS
    friend class helios::analytics::HDA_StateJSONReporter;
#endif
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
	/**
	 * @brief Specify the parallelization strategy
	 * @see ArgumentsParser::parseParallelizationStrategy
	 */
	int parallelizationStrategy;

	/**
	 * @brief Pulse thread pool
	 * @see PulseThreadPool
	 * @see PulseWarehouseThreadPool
	 */
	std::shared_ptr<PulseThreadPoolInterface> threadPool;
    /**
     * @brief Pulse task dropper
     * @see PulseTaskDropper
     */
    PulseTaskDropper taskDropper;

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
	 * @brief The handler for simulation steps, it also contains the discrete
	 *  time object that hanldes simulation frequency and time
	 */
	SimulationStepLoop stepLoop;

	/**
	 * @brief The callback frequency. It specifies how many steps must elapse
	 *  between consecutive callbacks
	 */
	size_t callbackFrequency = 0;

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
     * @brief Time corresponding to simulation start (nanoseconds)
     */
    std::chrono::nanoseconds timeStart_ns;

    /**
     * @brief Time corresponding to simulation start (currentGpsTime in
     *  nanoseconds)
     */
    double currentGpsTime_ns = 0;
    /**
     * @brief The time step for GPS time (in nanoseconds)
     * @see Simulation::currentGpsTime_ns
     */
    double stepGpsTime_ns = 0;
    /**
     * @brief Given fixed time start for GPS time as a string.
     *
     * Valid formats are either "YYYY-MM-DD hh:mm:ss" or POSIX timestamp.
     *
     * If it is an empty string "", then it means no fixed GPS time start must
     *  be considered
     */
    std::string fixedGpsTimeStart = "";
    /**
     * @brief Whether to use the legacy energy model (true) or not (false).
     * @see EnergyModel
     * @see BaseEnergyModel
     * @see ImprovedEnergyModel
     * @see ScanningDevice
     * @see Simulation::prepareSimulation
     */
    bool legacyEnergyModel = false;
    /**
     * @brief The report to generate reports about simulation
     * @see SimulationReporter
     */
    SimulationReporter reporter;

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

    std::shared_ptr<SimulationCycleCallback> callback = nullptr;

    /**
     * @brief Simulation player to handle the simulation loop from the outside.
     *
     * While the simulation loop handles all the legs of a survey, the
     *  simulation player handles the simulation loop, e.g., for a survey
     *  with repetitions the simulation player provides the logic to correctly
     *  launch the simulation loop correctly many times (one per repetition).
     *
     * @see SimulationPlayer
     * @see SwapOnRepeatHandler
     */
    std::unique_ptr<SimulationPlayer> simPlayer = nullptr;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simulation constructor
     * @param pulseThreadPoolInterface The thread pool to be used for
     *  parallel computation during simulation
     * @see PulseThreadPoolInterface
     * @see PulseThreadPool
     * @see PulseWarehouseThreadPool
     * @see PulseTaskDropper
     */
    Simulation(
        int const parallelizationStrategy,
        std::shared_ptr<PulseThreadPoolInterface> pulseThreadPoolInterface,
        int const chunkSize,
        std::string fixedGpsTimeStart="",
        bool const legacyEnergyModel=false
    );

    // ***  SIMULATION METHODS  *** //
    // **************************** //
    /**
     * @brief Prepare the simulation before starting its main loop
     * @param simFrequency_hz The simulation frequency in hertz
     */
    virtual void prepareSimulation(int simFrequency_hz);
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
     * @brief Handle the main simulation loop, i.e., the loop that governs
     *  the computations between the first and the last leg of the survey.
     * @return The number of iterations run in the simulation loop.
     */
    virtual void doSimLoop();
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
	 * @brief Handle simulation shutdown.
	 */
	virtual void shutdown();


	// ***  UTIL METHODS  *** //
	// ********************** //
    /**
     * @brief Compute the current GPS time (nanoseconds)
     * @return Current GPS time (nanoseconds)
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
    inline double getSimSpeedFactor() const {return this->mSimSpeedFactor;}

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
	inline std::shared_ptr<Scanner> getScanner() {return this->mScanner;}


	/**
	 * @brief Check if simulation is paused (true) or not (false)
	 * @return True if simulation is paused, false otherwise
	 * @see Simulation::mPaused
	 */
	inline bool isPaused() const {return this->mPaused;}
	/**
	 * @brief Check if simulation is stopped (true) or not (false)
	 * @return True if simulation is stopped, false otherwise
	 * @see Simulation::mStopped
	 */
	inline bool isStopped() const {return this->mStopped;}
	/**
	 * @brief Obtain simulation frequency
	 * @return Simulation frequency (hertz)
	 * @see Simulation::stepLoop
	 */
	inline size_t getSimFrequency() const {return stepLoop.getFrequency();}
	/**
	 * @brief Set simulation frequency
	 * @param simFrequency New simulation frequency (hertz)
	 * @see Simulation::stepLoop
	 */
	inline void setSimFrequency(size_t const simFrequency)
    {stepLoop.setFrequency(simFrequency);}
    /**
     * @brief Get the callback frequency. It is, how many steps must elapse
     *  between consecutive callbacks
     * @return Callback frequency
     * @see Simulation::callbackFrequency
     * @see Simulation::setCallbackFrequency
     */
    inline size_t getCallbackFrequency() const
    {return callbackFrequency;}
    /**
     * @brief Set the new callback frequency
     * @param callbackFrequency New callback frequency
     * @see Simulation::callbackFrequency
     * @see Simulation::getCallbackFrequency
     */
    inline void setCallbackFrequency(size_t const callbackFrequency)
    {this->callbackFrequency = callbackFrequency;}
    /**
     * @brief Get the simulation step loop of the simulation
     * @return The simulation's step loop
     * @see Simulation:stepLoop
     * @see SimulationStepLoop
     */
    SimulationStepLoop & getStepLoop()
    {return stepLoop;}
};
