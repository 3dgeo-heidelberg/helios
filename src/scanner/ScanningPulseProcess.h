#pragma once

#include <Measurement.h>
#include <Rotation.h>
#include <PulseTaskFactory.h>
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
     * @brief The pulse task factory to build pulse tasks
     * @see PulseTaskFactory
     */
    PulseTaskFactory ptf;
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
    );
    virtual ~ScanningPulseProcess() = default;

    // ***  PULSE COMPUTATION  *** //
    // *************************** //
    /**
     * @brief Handle pulse computation whatever it is single thread based
     * or thread pool based
     * @param legIndex Index of current leg
     * @param absoluteBeamOrigin Absolute position of beam origin
     * @param absoluteBeamAttitude Beam attitude
     * @param currentGpsTime Current GPS time (nanoseconds)
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

    // *** GETTERs and SETTERs  *** //
    // **************************** //
    /**
     * @brief Obtain the scanner's detector
     * @return Scanner's detector
     * @see ScanningPulseProcess::detector
     */
    inline std::shared_ptr<AbstractDetector> & getDetector() const
    {return detector;}
    /**
     * @brief Obtain the scanner's current pulse number
     * @return Scanner's current pulse number
     * @see ScanningPulseProcess::currentPulseNumber
     */
    inline int & getCurrentPulseNumber() const {return currentPulseNumber;}
    /**
     * @brief Obtain the scanner's write waveform flag
     * @return Scanner's write waveform flag
     * @see ScanningPulseProcess::writeWaveform
     */
    inline bool & isWriteWaveform() const {return writeWaveform;}
    /**
     * @brief Obtain the scanner's calc echowidth flag
     * @return Scanner's calc echowidth flag
     * @see ScanningPulseProcess::calcEchowidth
     */
    inline bool & isCalcEchowidth() const {return calcEchowidth;}
    /**
     * @brief Obtain the scanner's all measurements vector
     * @return Scanner's all measurements vector
     * @see ScanningPulseProcess::allMeasurements
     */
    inline std::shared_ptr<std::vector<Measurement>> & getAllMeasurements(
    )const
    {return allMeasurements;}
    /**
     * @brief Obtain the scanner's all measurements mutex
     * @return Scanner's all measurements mutex
     * @see ScanningPulseProcess::allMeasurementsMutex
     */
    inline std::shared_ptr<std::mutex> & getAllMeasurementsMutex() const
    {return allMeasurementsMutex;}
    /**
     * @brief Obtain the scanner's cycle measurements vector
     * @return Scanner's cycle measurements vector
     * @see ScanningPulseProcess::cycleMeasurements
     */
    inline std::shared_ptr<std::vector<Measurement>> & getCycleMeasurements(
    )const
    {return cycleMeasurements;}
    /**
     * @brief Obtain the scanner's cycle measurements mutex
     * @return Scanner's cycle measurements mutex
     * @see ScanningPulseProcess::cycleMeasurementsMutex
     */
    inline std::shared_ptr<std::mutex> &getCycleMeasurementsMutex() const
    {return cycleMeasurementsMutex;}
};