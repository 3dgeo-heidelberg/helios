#ifdef PYTHON_BINDING

#include <pybinds/PyDetectorWrapper.h>
#include <pybinds/PyScannerWrapper.h>

using namespace pyhelios;

PyDetectorWrapper * PyScannerWrapper::getPyDetectorWrapper(){
    return new PyDetectorWrapper(scanner.getDetector());
}


#endif
