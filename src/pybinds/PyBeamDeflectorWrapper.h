#pragma once

#include <AbstractBeamDeflector.h>
#include <scanner/beamDeflector/ConicBeamDeflector.h>
#include <scanner/beamDeflector/FiberArrayBeamDeflector.h>
#include <scanner/beamDeflector/OscillatingMirrorBeamDeflector.h>
#include <scanner/beamDeflector/PolygonMirrorBeamDeflector.h>
#include <scanner/beamDeflector/RisleyBeamDeflector.h>

#include <memory>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for AbstractBeamDeflector class
 *
 * @see AbstractBeamDeflector
 */
class PyBeamDeflectorWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    AbstractBeamDeflector &beamDeflector;

    // ***  CONSTRUCTION  *** //
    // ********************** //
    PyBeamDeflectorWrapper(
        std::shared_ptr<AbstractBeamDeflector> beamDeflector
    ) : beamDeflector(*beamDeflector) {}
    virtual ~PyBeamDeflectorWrapper(){}

	// ***  GETTERS and SETTERS  *** //
	// ***************************** //
	inline double getScanFreqMax()
	    {return beamDeflector.cfg_device_scanFreqMax_Hz;}
	inline void setScanFreqMax(double scanFreqMax_Hz)
        {beamDeflector.cfg_device_scanFreqMax_Hz = scanFreqMax_Hz;}
	inline double getScanFreqMin()
        {return beamDeflector.cfg_device_scanFreqMin_Hz;}
    inline void setScanFreqMin(double scanFreqMin_Hz)
        {beamDeflector.cfg_device_scanFreqMin_Hz = scanFreqMin_Hz;}
    inline double getScanAngleMax()
        {return beamDeflector.cfg_device_scanAngleMax_rad;}
    inline void setScanAngleMax(double scanAngleMax)
        {beamDeflector.cfg_device_scanAngleMax_rad = scanAngleMax;}
    inline double getScanFreq()
        {return beamDeflector.cfg_device_scanFreqMin_Hz;}
    inline void setScanFreq(double scanFreq)
        {beamDeflector.cfg_device_scanFreqMin_Hz = scanFreq;}
    inline double getScanAngle()
        {return beamDeflector.cfg_setting_scanAngle_rad;}
    inline void setScanAngle(double scanAngle)
        {beamDeflector.cfg_setting_scanAngle_rad = scanAngle;}
    inline double getVerticalAngleMin()
        {return beamDeflector.cfg_setting_verticalAngleMin_rad;}
    inline void setVerticalAngleMin(double verticalAngleMin)
        {beamDeflector.cfg_setting_verticalAngleMin_rad = verticalAngleMin;}
    inline double getVerticalAngleMax()
        {return beamDeflector.cfg_setting_verticalAngleMax_rad;}
    inline void setVerticalAngleMax(double verticalAngleMax)
        {beamDeflector.cfg_setting_verticalAngleMax_rad = verticalAngleMax;}
    inline double getCurrentBeamAngle()
        {return beamDeflector.state_currentBeamAngle_rad;}
    inline void setCurrentBeamAngle(double currentBeamAngle)
        {beamDeflector.state_currentBeamAngle_rad = currentBeamAngle;}
    inline double getAngleDiff()
        {return beamDeflector.state_angleDiff_rad;}
    inline void setAngleDiff(double angleDiff)
        {beamDeflector.state_angleDiff_rad = angleDiff;}
    inline double getCachedAngleBetweenPulses()
        {return beamDeflector.cached_angleBetweenPulses_rad;}
    inline void setCachedAngleBetweenPulses(double angleBetweenPulses)
        {beamDeflector.cached_angleBetweenPulses_rad = angleBetweenPulses;}
    inline Rotation& getEmitterRelativeAttitude()
        {return beamDeflector.getEmitterRelativeAttitudeByReference();}
    inline std::string getOpticsType() const {
    return beamDeflector.getOpticsType();}
}
};

}
