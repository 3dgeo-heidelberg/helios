#pragma once

#ifdef PYTHON_BINDING

#include <PyDetectorWrapper.h>
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
    inline void setAccuracy(double accuracy)
        {detector.cfg_device_accuracy_m = accuracy;}
    inline double getRangeMin()
        {return detector.cfg_device_rangeMin_m;}
    inline void setRangeMin(double rangeMin)
        {detector.cfg_device_rangeMin_m = rangeMin;}
    inline double getLasScale()
        {return detector.lasScale;}
    inline void setLasScale(double lasScale)
        {detector.lasScale = lasScale;}
};

}

#endif