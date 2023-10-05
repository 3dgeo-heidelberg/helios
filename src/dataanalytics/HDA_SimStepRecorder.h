#ifdef DATA_ANALYTICS
#pragma once

#include <HDA_Recorder.h>
#include <HDA_RecordBuffer.h>
#include <sim/core/SurveyPlayback.h>

#include <string>
#include <memory>

namespace helios { namespace analytics{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The HeliosDataAnalytics recorder for simulation steps. It is, a class
 *  which records the data representing the state of simulation components at
 *  a certain simulation step.
 */
class HDA_SimStepRecorder : public HDA_Recorder {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The SurveyPlayback which simulation steps must be recorded
     */
    SurveyPlayback *sp;
    /**
     * @brief The record buffer for the x coordinate of the platform position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformPositionX;
    /**
     * @brief The record buffer for the y coordinate of the platform position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformPositionY;
    /**
     * @brief The record buffer for the z coordinate of the platform position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformPositionZ;
    /**
     * @brief The record buffer for the roll angle of the platform
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformRoll;
    /**
     * @brief The record buffer for the pitch angle of the platform
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformPitch;
    /**
     * @brief The record buffer for the yaw angle of the platform
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformYaw;
    /**
     * @brief The record buffer for the x coordinate of the platform's
     *  absolute mount position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformMountX;
    /**
     * @brief The record buffer for the y coordinate of the platform's
     *  absolute mount position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformMountY;
    /**
     * @brief The record buffer for the z coordinate of the platform's
     *  absolute mount position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformMountZ;
    /**
     * @brief The record buffer for the roll angle of the platform's absolute
     *  mount attitude
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformMountRoll;
    /**
     * @brief The record buffer for the pitch angle of the platform's absolute
     *  mount attitude
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformMountPitch;
    /**
     * @brief The record buffer for the yaw angle of the platform's absolute
     *  mount attitude
     */
    std::shared_ptr<HDA_RecordBuffer<double>> platformMountYaw;
    /**
     * @brief The record buffer for the x coordinate of the scanner position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerPositionX;
    /**
     * @brief The record buffer for the y coordinate of the scanner position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerPositionY;
    /**
     * @brief The record buffer for the z coordinate of the scanner position
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerPositionZ;
    /**
     * @brief The record buffer for the roll angle of the scanner
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerRoll;
    /**
     * @brief The record buffer for the pitch angle of the scanner
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerPitch;
    /**
     * @brief The record buffer for the yaw angle of the scanner
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerYaw;
    /**
     * @brief The record buffer for the roll angle of the scanner's head
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerHeadRoll;
    /**
     * @brief The record buffer for the pitch angle of the scanner's head
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerHeadPitch;
    /**
     * @brief The record buffer for the yaw angle of the scanner's head
     */
    std::shared_ptr<HDA_RecordBuffer<double>> scannerHeadYaw;
    /**
     * @brief The record buffer for the roll angle of the deflector's pulses
     */
    std::shared_ptr<HDA_RecordBuffer<double>> deflectorEmittingRoll;
    /**
     * @brief The record buffer for the pitch angle of the deflector's pulses
     */
    std::shared_ptr<HDA_RecordBuffer<double>> deflectorEmittingPitch;
    /**
     * @brief The record buffer for the yaw angle of the deflector's pulses
     */
    std::shared_ptr<HDA_RecordBuffer<double>> deflectorEmittingYaw;
    /**
     * @brief The record buffer for the x coordinate of the beam's origin
     */
    std::shared_ptr<HDA_RecordBuffer<double>> beamOriginX;
    /**
     * @brief The record buffer for the y coordinate of the beam's origin
     */
    std::shared_ptr<HDA_RecordBuffer<double>> beamOriginY;
    /**
     * @brief The record buffer for the z coordinate of the beam's origin
     */
    std::shared_ptr<HDA_RecordBuffer<double>> beamOriginZ;
    /**
     * @brief The record buffer for the roll angle of the beam
     */
    std::shared_ptr<HDA_RecordBuffer<double>> beamRoll;
    /**
     * @brief The record buffer for the pitch angle of the beam
     */
    std::shared_ptr<HDA_RecordBuffer<double>> beamPitch;
    /**
     * @brief The record buffer for the yaw angle of the beam
     */
    std::shared_ptr<HDA_RecordBuffer<double>> beamYaw;
    /**
     * @brief The record buffer for the main deflection angle (with no error)
     */
    std::shared_ptr<HDA_RecordBuffer<double>> exactDeflectionAngle;
    /**
     * @brief The record buffer for the main deflection angle (with error)
     */
    std::shared_ptr<HDA_RecordBuffer<double>> deflectionAngle;
    /**
     * @brief The record buffer for the head angle (with no error)
     */
    std::shared_ptr<HDA_RecordBuffer<double>> exactHeadAngle;
    /**
     * @brief The record buffer for the head angle (with error)
     */
    std::shared_ptr<HDA_RecordBuffer<double>> headAngle;

    /**
     * @brief The record buffer for the measurement error (used by sequential
     *  executions)
     */
    std::shared_ptr<HDA_RecordBuffer<double>> measErrSeq;
    /**
     * @brief The record buffer for the measurement error (used by parallel
     *  executions)
     */
    std::shared_ptr<HDA_RecordBuffer<double>> measErrPar;


public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a HDA_SimStepRecorder so the simulation process is written
     *  to data files in the directory specified through given path
     * @param sp The SurveyPlayback which simulation must be recorded
     * @param path Path of the directory where simulation records will be
     *  written
     */
    HDA_SimStepRecorder(
        SurveyPlayback *sp,
        std::string const &path
    ) :
        HDA_Recorder(path),
        sp(sp)
    {
        openBuffers();
    }

    virtual ~HDA_SimStepRecorder() {
        if(isAnyBufferOpen()) closeBuffers();
    }

    // ***  MAIN RECORD METHOD  *** //
    // **************************** //
    /**
     * @brief Handle all the records for the current simulation step.
     */
    virtual void record();
    /**
     * @brief Handle all the records that must be extracted after simulation
     *  has finished (it is named delayed record because it is postponed to
     *  until the simulation has finished)
     */
    virtual void delayedRecord();

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

protected:
    // ***  CONCRETE RECORD METHODS  *** //
    // ********************************* //
    /**
     * @brief Handle all the platform related records for the current
     *  simulation step
     */
    virtual void recordPlatform();
    /**
     * @brief Handle all the platform's absolute mount related records for the
     *  current simulation step
     */
    virtual void recordPlatformMount();
    /**
     * @brief Handle all the scanner related records for the current
     *  simulation step
     */
    virtual void recordScanner();
    /**
     * @brief Handle all the scanner's head related records for the current
     *  simulation step
     */
    virtual void recordScannerHead();
    /**
     * @brief Handle all the deflector related records for the current
     *  simulation step
     */
    virtual void recordDeflector();
    /**
     * @brief Handle all the beam related records for the current simulation
     *  step
     */
    virtual void recordBeam();
    /**
     * @brief Handle all the stochastic measurement related records for the
     *  current simulation step
     */
    virtual void recordStochastic();
};


}}

#endif