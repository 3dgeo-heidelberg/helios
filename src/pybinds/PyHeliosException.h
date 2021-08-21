#pragma once
#ifdef PYTHON_BINDING

#include <HeliosException.h>

namespace pyhelios{


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * Simple wrapper for HeliosException
 */
class PyHeliosException : public HeliosException{
public:
    PyHeliosException(std::string const msg = "") : HeliosException(msg) {}
};

}

#endif