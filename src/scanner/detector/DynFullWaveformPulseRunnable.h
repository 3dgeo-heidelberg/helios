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
     * @param raycaster The temporal KDGrove for the dynamic full waveform
     *  pulse runnable
     * @see FullWaveformPulseRunnable::FullWaveformPulseRunnable
     * @see DynFullWaveformPulseRunnable::raycaster
     */
    DynFullWaveformPulseRunnable(
        std::shared_ptr<KDGroveRaycaster> raycaster,
        std::shared_ptr<Scanner> scanner,
        SimulatedPulse const &pulse
    ) :
        FullWaveformPulseRunnable(scanner, pulse),
        raycaster(raycaster)
    {}
    ~DynFullWaveformPulseRunnable() override = default;

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