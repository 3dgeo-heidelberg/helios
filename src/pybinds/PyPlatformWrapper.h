#pragma once

#ifdef PYTHON_BINDING

#include <PyNoiseSourceWrapper.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Platform class
 */
class PyPlatformWrapper {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    Platform &platform;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyPlatformWrapper(Platform &platform) : platform(platform) {}

    virtual ~PyPlatformWrapper() = default;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    double getLastCheckZ() { return platform.lastCheckZ; }

    void setLastCheckZ(double checkZ) { platform.lastCheckZ = checkZ; }

    double getDmax() { return platform.dmax; }

    void setDmax(double dmax) { platform.dmax = dmax; }

    double getMovePerSec() { return platform.cfg_settings_movePerSec_m; }

    void setMovePerSec(double movePerSec) { platform.cfg_settings_movePerSec_m = movePerSec; }

    bool isOnGround() { return platform.onGround; }

    void setOnGround(bool onGround) { platform.onGround = onGround; }

    bool isStopAndTurn() { return platform.stopAndTurn; }

    void setStopAndTurn(bool stopAndTurn) { platform.stopAndTurn = stopAndTurn; }

    bool isOrientationOnLegInit() { return platform.mSetOrientationOnLegInit; }

    void setOrientationOnLegInit(bool setOrientationOnLegInit) { platform.mSetOrientationOnLegInit = setOrientationOnLegInit; }

    PyNoiseSourceWrapper *getPositionXNoiseSource() {
        if (platform.positionXNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(*platform.positionXNoiseSource);
    }

    PyNoiseSourceWrapper *getPositionYNoiseSource() {
        if (platform.positionYNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(*platform.positionYNoiseSource);
    }

    PyNoiseSourceWrapper *getPositionZNoiseSource() {
        if (platform.positionZNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(*platform.positionZNoiseSource);
    }

    PyNoiseSourceWrapper *getAttitudeXNoiseSource(){
        if(platform.attitudeXNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(*platform.attitudeXNoiseSource);
    }

    PyNoiseSourceWrapper *getAttitudeYNoiseSource(){
        if(platform.attitudeYNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(*platform.attitudeYNoiseSource);
    }

    PyNoiseSourceWrapper *getAttitudeZNoiseSource(){
        if(platform.attitudeZNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(*platform.attitudeZNoiseSource);
    }

    PythonDVec3 * getRelativePosition()
        {return new PythonDVec3(&platform.cfg_device_relativeMountPosition);}
    Rotation & getRelativeAttitude()
        {return platform.cfg_device_relativeMountAttitude;}
    PythonDVec3 * getLastGroundCheck()
        {return new PythonDVec3(&platform.lastGroundCheck);}
    PythonDVec3 * getNextWaypointPosition()
        {return new PythonDVec3(&platform.targetWaypoint);}
    PythonDVec3 * getPositionPython()
        {return new PythonDVec3(&platform.position);}
    Rotation & getAttitudePython()
        {return platform.attitude;}
    PythonDVec3 * getCachedAbsolutePosition()
        {return new PythonDVec3(&platform.cached_absoluteMountPosition);}
    Rotation & getCachedAbsoluteAttitude()
        {return platform.cached_absoluteMountAttitude;}
    PythonDVec3 * getCachedCurrentDir()
        {return new PythonDVec3(&platform.cached_dir_current);}
    PythonDVec3 * getCachedCurrentDirXY()
        {return new PythonDVec3(&platform.cached_dir_current_xy);}
    PythonDVec3 * getCachedVectorToTarget()
        {return new PythonDVec3(&platform.cached_vectorToTarget);}
    PythonDVec3 * getCachedVectorToTargetXY()
        {return new PythonDVec3(&platform.cached_vectorToTarget_xy);}
};

#endif