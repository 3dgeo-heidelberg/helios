#pragma once

#ifdef PYTHON_BINDING

#include <PyDetectorWrapper.h>
#include <filems/facade/FMSFacade.h>
using helios::filems::FMSFacade;
#include <AbstractDetector.h>

#include <memory>


namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for AbstractDetector class
 *
 * @see AbstractDetector
 */
class PyDetectorWrapper {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    AbstractDetector &detector;

    // ***  CONSTRUCTION  *** //
    // ********************** //
    PyDetectorWrapper(
        std::shared_ptr<AbstractDetector> detector
    ) :
        detector(*detector) {}

    virtual ~PyDetectorWrapper() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    inline double getAccuracy()
        {return detector.cfg_device_accuracy_m;}
    inline void setAccuracy(double const accuracy)
        {detector.cfg_device_accuracy_m = accuracy;}
    inline double getRangeMin()
        {return detector.cfg_device_rangeMin_m;}
    inline void setRangeMin(double const rangeMin)
        {detector.cfg_device_rangeMin_m = rangeMin;}
    inline double getRangeMax()
        {return detector.cfg_device_rangeMax_m;}
    inline void setRangeMax(double const rangeMax)
        {detector.cfg_device_rangeMax_m = rangeMax;}
    inline double getLasScale()
        {return detector.getFMS()->write.getMeasurementWriterLasScale();}
    inline void setLasScale(double const lasScale)
        {detector.getFMS()->write.setMeasurementWriterLasScale(lasScale);}
};

}

#endif