#pragma once

#include <PyMeasurementWrapper.h>
#include <PyHeliosUtils.h>
#include <vector>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for std::vector<Measurement> class
 *
 * @see std::vector
 * @see PyWrapperMeasurement
 * @see Measurement
 */
class PyMeasurementVectorWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    std::vector<Measurement> allMeasurements;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyMeasurementVectorWrapper(std::vector<Measurement> &allMeasurements) :
        allMeasurements(allMeasurements) {}
    virtual ~PyMeasurementVectorWrapper() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    PyMeasurementWrapper * get(long index){
        return new PyMeasurementWrapper(allMeasurements[
            PyHeliosUtils::handlePythonIndex(index, allMeasurements.size())
        ]);
    }
    void erase(long index){
        allMeasurements.erase(
            allMeasurements.begin() +
            PyHeliosUtils::handlePythonIndex(index, allMeasurements.size())
        );
    }
    size_t length() {return allMeasurements.size();}

};

}
