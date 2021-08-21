#pragma once

#ifdef PYTHON_BINDING

#include <NoiseSource.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for NoiseSource abstract class
 *
 * @see NoiseSource
 */
class PyNoiseSourceWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    NoiseSource<double> &ns;

    // ***  CONSTRUCTION  *** //
    // ********************** //
    PyNoiseSourceWrapper(NoiseSource<double> &ns) : ns(ns) {}
    virtual ~PyNoiseSourceWrapper(){}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    inline double getClipMin() {return ns.getClipMin();}
    inline void setClipMin(double clipMin) {ns.setClipMin(clipMin);}
    inline double getClipMax() {return ns.getClipMax();}
    inline void setClipMax(double clipMax) {ns.setClipMax(clipMax);}
    inline bool isEnabled() {return ns.isClipEnabled();}
    inline void setEnabled(bool enabled) {ns.setClipEnabled(enabled);}
    inline bool isFixedValueEnabled() {return ns.isFixedValueEnabled();}
    inline unsigned long getFixedLifespan() {return ns.getFixedLifespan();}
    inline void setFixedLifespan(unsigned long fixedLifespan)
        {ns.setFixedLifespan(fixedLifespan);}
    inline unsigned long getFixedValueRemainingUses()
        {return ns.getFixedValueRemainingUses();}
    inline void setFixedValueRemainingUses(unsigned long remainingUses)
        {ns.setFixedValueRemainingUses(remainingUses);}
    double next(){return ns.next();}
};

}

#endif