#pragma once

#include <FullWaveformPulseRunnable.h>
#include <KDGroveRaycaster.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Derived class extending FullWaveFormPulseRunnable to handle dynamic
 *  moving objects
 * @see FullWaveformPulseRunnable
 */
class DynFullWaveformPulseRunnable : public FullWaveformPulseRunnable {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The temporal KDGrove raycaster to compute pulse simulation
     */
    std::shared_ptr<KDGroveRaycaster> raycaster;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Base constructor for dynamic full waveform pulse runnable
     * @param tkdg The temporal KDGrove for the dynamic full waveform pulse
     *  runnable
     * @see FullWaveformPulseRunnable::FullWaveformPulseRunnable
     * @see DynFullWaveformPulseRunnable::tkdg
     */
    DynFullWaveformPulseRunnable(
        std::shared_ptr<KDGroveRaycaster> raycaster,
        std::shared_ptr<FullWaveformPulseDetector> detector,
        glm::dvec3 absoluteBeamOrigin,
        Rotation absoluteBeamAttitude,
        int currentPulseNum,
        double currentGpsTime,
        bool writeWaveform,
        bool calcEchowidth,
        std::vector<Measurement> * allMeasurements,
        std::mutex * allMeasurementsMutex,
        std::vector<Measurement> * cycleMeasurements,
        std::mutex * cycleMeasurementsMutex,
        unsigned int legIndex
    ) :
        FullWaveformPulseRunnable(
            detector,
            absoluteBeamOrigin,
            absoluteBeamAttitude,
            currentPulseNum,
            currentGpsTime,
            writeWaveform,
            calcEchowidth,
            allMeasurements,
            allMeasurementsMutex,
            cycleMeasurements,
            cycleMeasurementsMutex,
            legIndex
        ),
        raycaster(raycaster)
    {}
    virtual ~DynFullWaveformPulseRunnable() = default;

    // ***  ASSISTANCE METHODS  *** //
    // **************************** //
    /**
     * @brief Find intersections considering there are dynamic moving objects
     *  involved
     * @see FullWaveformPulserunnable::findIntersection
     */
    shared_ptr<RaySceneIntersection> findIntersection(
        vector<double> const &tMinMax,
        glm::dvec3 const &o,
        glm::dvec3 const &v
    ) const override;
};