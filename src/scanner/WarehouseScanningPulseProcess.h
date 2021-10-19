#pragma once

#include <scanner/ScanningPulseProcess.h>
#include <PulseTaskDropper.h>
#include <PulseWarehouseThreadPool.h>

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a scanning pulse process which works with a
 *  pulse task dropper based on budding task dropper and a pulse warehouse
 *  thread pool based on an asynchronous warehouse thread pool
 * @see PulseTaskDropper
 * @see PulseWarehouseThreadPool
 */
class WarehouseScanningPulseProcess : public ScanningPulseProcess {
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
    PulseWarehouseThreadPool &pool;
    /**
     * @brief Pulse computation handling function. It will be configured at
     *  construction depending on thread pool size to assign sequential or
     *  parallel computing method as corresponds
     */
    std::function<void(
        unsigned int const, glm::dvec3 &, Rotation &, double const
    )> handler;

    /**
     * @brief Alpha-Prime Matrix for sequential executions (either single
     *  thread or main thread workload)
     * @see MarquardtFitter::ALPHA_PRIME
     */
    std::vector<std::vector<double>> apMatrix;
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

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for warehouse scanning pulse process
     * @param dropper The task dropper used to handle job chunks
     * @param pool Thread pool to be used to handle multi threading pulse
     *  computation
     * @param randGen1 First randomness generator for single thread
     * @param randGen2 Second randomness generator for single thread mode
     * @param intersectionHandlingNoiseSource Uniform noise source for single
     *  thread mode
     * @see ScanningPulseProcess::ScanningPulseProcess
     */
    WarehouseScanningPulseProcess(
        std::shared_ptr<AbstractDetector> &detector,
        int &currentPulseNumber,
        bool &writeWaveform,
        bool &calcEchowidth,
        std::shared_ptr<std::vector<Measurement>> &allMeasurements,
        std::shared_ptr<std::mutex> &allMeasurementsMutex,
        std::shared_ptr<std::vector<Measurement>> &cycleMeasurements,
        std::shared_ptr<std::mutex> &cycleMeasurementsMutex,
        PulseTaskDropper &dropper,
        PulseWarehouseThreadPool &pool,
        RandomnessGenerator<double> &randGen1,
        RandomnessGenerator<double> &randGen2,
        UniformNoiseSource<double> &intersectionHandlingNoiseSource
    );
    virtual ~WarehouseScanningPulseProcess() = default;

    // ***  PULSE COMPUTATION  *** //
    // *************************** //
    /**
     * @brief Implementation of handle pulse computation method for the pair
     *  budding task dropper and pulse warehouse thread pool
     * @see ScanningPulseProcess::handlePulseComputation
     * @see PulseTaskDropper
     * @see PulseWarehouseThreadPool
     * @see WarehouseScanningPulseProcess::handlePulseComputationSequential
     * @see WarehouseScanningPulseProcess::handlePulseComputationParallel
     */
    inline void handlePulseComputation(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    ) override {
        this->handler(
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
     * @brief Handle shutdown of warehouse thread pool (final join)
     * @see WarehouseThreadPool::finish
     * @see WarehouseThreadPool::finalJoin
     */
    void onSimulationFinished() override;

protected:
    // ***  INNER PULSE COMPUTATION  *** //
    // ********************************* //
    /**
     * @brief Handle sequential computation of scanning pulses
     * @see WarehouseScanningPulseProcess::handlePulseComputation
     * @see WarehouseScanningPulseProcess::handlePulseComputationParallel
     */
    virtual void handlePulseComputationSequential(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    );
    /**
     * @brief Handle parallel computation of scanning pulses using a warehouse
     *  of task-chunks based strategy
     * @see WarehouseScanningPulseProcess::handlePulseComputation
     * @see WarehouseScanningPulseProcess::handlePulseComputationSequential
     */
    virtual void handlePulseComputationParallel(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    );
};
