#pragma once

#include <Measurement.h>
#include <Rotation.h>
class AbstractDetector;

#include <glm/glm.hpp>

#include <mutex>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class defining the scanning pulse process interface
 */
class ScanningPulseProcess {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
	 * @brief Scanner's detector
     * @see Scanner::detector
	 * @see AbstractDetector
	 */
    std::shared_ptr<AbstractDetector> &detector;
    /**
     * @brief Reference to scanner's current pulse number
     * @see Scanner::state_currentPulseNumber
     */
    int &currentPulseNumber;
    /**
     * @brief Reference to scanner's write waveform flag
     * @see Scanner::writeWaveform
     */
    bool &writeWaveform;
    /**
     * @brief Reference to scanner's calc echowidth flag
     * @see Scanner::calcEchowidth
     */
    bool &calcEchowidth;
    /**
     * @brief Reference to scanner's all measurements vector
     * @see Scanner::allMeasurements
     */
    std::shared_ptr<std::vector<Measurement>> &allMeasurements;
    /**
     * @brief Reference to scanner's all measurements mutex
     * @see Scanner::allMeasurementsMutex
     */
    std::shared_ptr<std::mutex> &allMeasurementsMutex;
    /**
     * @brief Reference to scanner's cycle measurements vector
     * @see Scanner::cycleMeasurements
     */
    std::shared_ptr<std::vector<Measurement>> &cycleMeasurements;
    /**
     * @brief Reference to scanner's cycle measurements mutex
     * @see Scanner::cycleMeasurementsMutex
     */
    std::shared_ptr<std::mutex> &cycleMeasurementsMutex;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for scanning pulse process
     */
    ScanningPulseProcess(
        std::shared_ptr<AbstractDetector> &detector,
        int &currentPulseNumber,
        bool &writeWaveform,
        bool &calcEchowidth,
        std::shared_ptr<std::vector<Measurement>> &allMeasurements,
        std::shared_ptr<std::mutex> &allMeasurementsMutex,
        std::shared_ptr<std::vector<Measurement>> &cycleMeasurements,
        std::shared_ptr<std::mutex> &cycleMeasurementsMutex
    ) :
        detector(detector),
        currentPulseNumber(currentPulseNumber),
        writeWaveform(writeWaveform),
        calcEchowidth(calcEchowidth),
        allMeasurements(allMeasurements),
        allMeasurementsMutex(allMeasurementsMutex),
        cycleMeasurements(cycleMeasurements),
        cycleMeasurementsMutex(cycleMeasurementsMutex)
    {}
    virtual ~ScanningPulseProcess() = default;

    // ***  PULSE COMPUTATION  *** //
    // *************************** //
    /**
     * @brief Handle pulse computation whatever it is single thread based
     * or thread pool based
     * @param legIndex Index of current leg
     * @param absoluteBeamOrigin Absolute position of beam origin
     * @param absoluteBeamAttitude Beam attitude
     * @param currentGpsTime Current GPS time (milliseconds)
     */
    virtual void handlePulseComputation(
        unsigned int const legIndex,
        glm::dvec3 &absoluteBeamOrigin,
        Rotation &absoluteBeamAttitude,
        double const currentGpsTime
    ) = 0;
    /**
     * @brief Handle behavior of scanning pulse process once current leg has
     *  been completed. It is useful mainly when scanning pulses are computed
     *  in a parallel way, so pending tasks can be adequately handled.
     *
     * Default implementation does nothing. Therefore, any concrete class
     *  providing an implementation of the ScanningPulseProcess interface must
     *  override this method if it needs to handle anything at on leg complete.
     */
    virtual inline void onLegComplete() {}
    /**
     * @brief Handle behavior of scanning pulse process once simulation has
     *  finished.
     *
     * Default implementation does nothing. Therefore, any concrete class
     *  providing an implementation of the ScanningPulseProcess interface must
     *  override this method if it needs to handle anything at on simulation
     *  finished.
     */
    virtual inline void onSimulationFinished() {}
};