#pragma once

#ifdef PYTHON_BINDING

#include <Trajectory.h>
#include <PythonDVec3.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Trajectory class
 *
 * @see Trajectory
 * @see PyTrajectoryVectorWrapper
 */

class PyTrajectoryWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    Trajectory &t;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyTrajectoryWrapper(Trajectory &t) : t(t) {}
    virtual ~PyTrajectoryWrapper() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    long getGpsTime() {return t.gpsTime;}
    void setGpsTime(long gpsTime) {t.gpsTime = gpsTime;}
    PythonDVec3 *getPosition() {return new PythonDVec3(t.position);}
    void setPosition(double x, double y, double z)
        {t.position = glm::dvec3(x, y, z);}
    double getRoll() {return t.roll;}
    void setRoll(double roll) {t.roll = roll;}
    double getPitch() {return t.pitch;}
    void setPitch(double pitch) {t.pitch = pitch;}
    double getYaw() {return t.yaw;}
    void setYaw(double yaw) {t.yaw = yaw;}
};

}

#endif