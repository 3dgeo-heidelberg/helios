#pragma once

#include <scanner/ScanningPulseProcess.h>
#include <PulseTaskDropper.h>
#include <PulseThreadPool.h>
#include <TimeWatcher.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a scanning pulse process which works with a
 *  pulse task dropper based on budding task dropper and a pulse thread pool
 *  based on an asynchronous resource thread pool
 * @see PulseTaskDropper
 * @see PulseThreadPool
 */
class BuddingScanningPulseProcess : public ScanningPulseProcess {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The task dropper used to handle job chunks
     */
    PulseTaskDropper &dropper;
    /**
     * @brief Thread pool to be used to handle multi threading pulse
     *  computation
     */
    PulseThreadPool &pool;
    /**
     * @brief Pulse computation handling function. It will be configured at
     *  construction depending on thread pool size to assign sequential or
     *  parallel computing method as corresponds
     */
    std::function<void(
        unsigned int const, glm::dvec3 &, Rotation &, double const
    )> handler;

    /**
     * @brief First randomness generator for single thread mode
     * @see Scanner::randGen1
     */
    RandomnessGenerator<double> &randGen1;
    /**
     * @brief Second randomness generator for single thread mode
     * @see Scanner::randGen2
     */
    RandomnessGenerator<double> &randGen2;
    /**
     * @brief Uniform noise source for single thread mode
     * @see Scanner::intersectionHandlingNoiseSource
     */
    UniformNoiseSource<double> &intersectionHandlingNoiseSource;

    /**
	 * @brief Length in nanoseconds of the last idle thread time interval
	 */
    long lastIdleNanos = 0;
    /*
	 * @brief Threshold so idle times which are below its value are not
	 *  considered. Instead, they are discarded as a non trustable measurement.
	 *  It is given in nanoseconds.
	 */
    long const idleTh = 100000;
    /**
	 * @brief Tolerance so idle times differences below this threshold will
	 *  not change sign of budding task dropper and neither last idle time.
	 *  It is given in nanoseconds.
	 */
    long const idleEps = 100000;

    /**
	 * @brief Scan idle timer to work with thread pool idle time
	 * @see PulseThreadPool::idleTimer
	 */
    TimeWatcher idleTimer;

#ifdef BUDDING_METRICS
#include <fstream>
    std::ofstream ofsBudding;
#endif

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for budding scanning pulse process
     * @param dropper The task dropper used to handle job chunks
     * @param pool Thread pool to be used to handle multi threading pulse
     *  computation
     * @param randGen1 First randomness generator for single thread
     * @param randGen2 Second randomness generator for single thread mode
     * @param intersectionHandlingNoiseSource Uniform noise source for single
     *  thread mode
     * @see ScanningPulseProcess::ScanningPulseProcess
     */
    BuddingScanningPulseProcess(
        std::shared_ptr<AbstractDetector> &detector,
        int &currentPulseNumber,
        bool &writeWaveform,
        bool &calcEchowidth,
        std::shared_ptr<std::vector<Measurement>> &allMeasurements,
        std::shared_ptr<std::mutex> &allMeasurementsMutex,
        std::shared_ptr<std::vector<Measurement>> &cycleMeasurements,
        std::shared_ptr<std::mutex> &cycleMeasurementsMutex,
        PulseTaskDropper &dropper,
        PulseThreadPool &pool,
        RandomnessGenerator<double> &randGen1,
        RandomnessGenerator<double> &randGen2,
        UniformNoiseSource<double> &intersectionHandlingNoiseSource,
        bool const dynamic=true
    );
    virtual ~BuddingScanningPulseProcess() = default;

    // ***  PULSE COMPUTATION  *** //
    // *************************** //
    /**
     * @brief Implementation of handle pulse computation method for the pair
     *  budding task dropper and pulse thread pool
     * @see ScanningPulseProcess::handlePulseComputation
     * @see PulseTaskDropper
     * @see PulseThreadPool
     * @see BuddingScanningPulseProcess::handlePulseComputationSequential
     * @see BuddingScanningPulseProcess::handlePulseComputationParallel
     */
    inline void handlePulseComputation(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    ) override {
        return this->handler(
            legIndex,
            absoluteBeamOrigin,
            absoluteBeamAttitude,
            currentGpsTime
        );
    }
    /**
     * @brief Handle sequential computation of a chunk of pulses through task
     *  dropper
     */
    void onLegComplete() override;
    /**
     * @brief Handle closing of output file stream for budding metrics when
     *  it is called with -DBUDDING_METRICS=1
     * @see ofsBudding
     */
    void onSimulationFinished() override;

protected:
    // ***  INNER PULSE COMPUTATION  *** //
    // ********************************* //
    /**
     * @brief Handle sequential computation of scanning pulses
     * @see BuddingScanningPulseProcess::handlePulseComputation
     * @see BuddingScanningPulseProcess::handlePulseComputationParallelDynamic
     * @see BuddingScanningPulseProcess::handlePulseComputationParallelStatic
     */
    virtual void handlePulseComputationSequential(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    );
    /**
     * @brief Handle parallel computation of scanning pulses using a dynamic
     *  chunk-size based strategy
     * @see BuddingScanningPulseProcess::handlePulseComputation
     * @see BuddingScanningPulseProcess::handlePulseComputationSequential
     * @see BuddingScanningPulseProcess::handlePulseComputationParallelStatic
     */
    virtual void handlePulseComputationParallelDynamic(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    );
    /**
     * @brief Handle a parallel computation of scanning pulse using a static
     *  chunk-size based strategy
     * @see BuddingScanningPulseProcess::handlePulseComputation
     * @see BuddingScanningPulseProcess::handlePulseComputationSequential
     * @see BuddingScanningPulseProcess::handlePulseComputationParallelDynamic
     */
    virtual void handlePulseComputationParallelStatic(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    );

};