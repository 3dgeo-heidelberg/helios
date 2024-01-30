#pragma once

#include <string>
#include <Measurement.h>
#include <PythonDVec3.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Measurement class
 *
 * @see Measurement
 */
class PyMeasurementWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    Measurement &m;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyMeasurementWrapper(Measurement &m) : m(m) {}
    virtual ~PyMeasurementWrapper() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    std::string getHitObjectId() {return m.hitObjectId;}
    void setHitObjectId(std::string const hitObjectId)
        {m.hitObjectId = hitObjectId;}
    PythonDVec3 *getPosition() {return new PythonDVec3(m.position);}
    void setPosition(double x, double y, double z)
        {m.position = glm::dvec3(x, y, z);}
    PythonDVec3 *getBeamDirection() {return new PythonDVec3(m.beamDirection);}
    void setBeamDirection(double x, double y, double z)
        {m.beamDirection = glm::dvec3(x, y, z);}
    PythonDVec3 *getBeamOrigin() {return new PythonDVec3(m.beamOrigin);}
    void setBeamOrigin(double x, double y, double z)
        {m.beamOrigin = glm::dvec3(x, y, z);}
    double getDistance() {return m.distance;}
    void setDistance(double distance) {m.distance = distance;}
    double getIntensity() {return m.intensity;}
    void setIntensity(double intensity) {m.intensity = intensity;}
    double getEchoWidth() {return m.echo_width;}
    void setEchoWidth(double echoWidth) {m.echo_width = echoWidth;}
    int getReturnNumber() {return m.returnNumber;}
    void setReturnNumber(int returnNumber) {m.returnNumber = returnNumber;}
    int getPulseReturnNumber() {return m.pulseReturnNumber;}
    void setPulseReturnNumber(double pulseReturnNumber)
        {m.pulseReturnNumber = pulseReturnNumber;}
    int getFullwaveIndex() {return m.fullwaveIndex;}
    void setFullwaveIndex(int fullwaveIndex) {m.fullwaveIndex = fullwaveIndex;}
    int getClassification() {return m.classification;}
    void setClassification(int classification)
        {m.classification = classification;}
    long getGpsTime() {return m.gpsTime;}
    void setGpsTime(long gpsTime) {m.gpsTime = gpsTime;}
};

}
