#include <pybinds/PyDetectorWrapper.h>
#include <pybinds/PyScannerWrapper.h>

using namespace pyhelios;

PyDetectorWrapper * PyScannerWrapper::getPyDetectorWrapper(){
    return new PyDetectorWrapper(scanner.getDetector());
}

PyDetectorWrapper * PyScannerWrapper::getPyDetectorWrapper(size_t const idx){
    return new PyDetectorWrapper(scanner.getDetector(idx));
}
