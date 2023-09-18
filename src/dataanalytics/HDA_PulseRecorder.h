#ifdef DATA_ANALYTICS
#pragma once

#include <HDA_Recorder.h>
#include <HDA_RecordBuffer.h>

#include <string>
#include <memory>
#include <mutex>

namespace helios { namespace analytics{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The HeliosDataAnalytics recorder for pulses (more concretely, pulse
 *  tasks / runnables). It is a class which records relevant data from the
 *  many pulse computations.
 * @see FullWaveformPulseRunnable
 */
class HDA_PulseRecorder : public HDA_Recorder{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The vector which components are variables involved on a
     *  particular intensity calculation for a given subray.
     *
     * [0] -> Incidence angle in radians.
     *
     * [1] -> The target range in meters, i.e., the distance between the beam's
     *  origin and the intersection point.
     *
     * [2] -> The target area in squared meters.
     *
     * [3] -> The radius in meters, i.e., the distance between the beam's
     *  center line and the intersection point.
     *
     * [4] -> The bidirectional reflectance function (BDRF).
     *
     * [5] -> The cross-section in squared meters.
     *
     * [6] -> The calculated received power, i.e., intensity.
     */
    std::shared_ptr<HDA_RecordBuffer<std::vector<double>>> intensityCalc;

    /**
     * @brief The mutex to handle concurrent writes to the buffers related to
     *  intensity calculation.
     */
    std::mutex intensityCalcMutex;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a HDA_PulseRecorder so pulse computations are written to
     *  data files in the directory specified through given path.
     * @param path Path of the directory where simulation records will be
     *  written.
     */
    HDA_PulseRecorder(std::string const &path) : HDA_Recorder(path){
        openBuffers();
    }

    virtual ~HDA_PulseRecorder() {
        if(isAnyBufferOpen()) closeBuffers();
    }

    // ***  RECORDER METHODS  *** //
    // ************************** //
    /**
     * @brief Check whether there are opened buffers or not
     * @return True if there is at least one opened buffer, false if there is
     *  not even a single opened buffer
     */
    bool isAnyBufferOpen();
    /**
     * @brief Open all the record buffers so the HDA_SimStepRecord can record
     */
    void openBuffers();
    /**
     * @brief Close all the record buffers. Once it is done, the
     *  HDA_SimStepRecorder will not be able to properly handle any new
     *  record.
     */
    void closeBuffers();


    // ***  RECORD METHODS  *** //
    // ************************ //
    /**
     * @brief Handle all the records for the current simulation step.
     */
    virtual void recordIntensityCalculation(
        double const incidenceAngle_rad,
        double const targetRange_m,
        double const targetArea_m2,
        double const radius_m,
        double const bdrf,
        double const crossSection,
        double const receivedPower
    );

};

}}
#endif