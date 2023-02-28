#pragma once

#ifdef PYTHON_BINDING

#include <PyMeasurementVectorWrapper.h>
#include <PyTrajectoryVectorWrapper.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Python wrapper for helios output
 *
 * @see PyMeasurementVectorWrapper
 * @see PyTrajectoryVectorWrapper
 */
class PyHeliosOutputWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    PyMeasurementVectorWrapper measurements;
    PyTrajectoryVectorWrapper trajectories;
    bool finished;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyHeliosOutputWrapper(
        std::shared_ptr<std::vector<Measurement>> measurements,
        std::shared_ptr<std::vector<Trajectory>> trajectories,
        bool finished
    ) :
        measurements(PyMeasurementVectorWrapper(*measurements)),
        trajectories(PyTrajectoryVectorWrapper(*trajectories)),
        finished(finished)
    {}
    PyHeliosOutputWrapper(
        std::vector<Measurement> &measurements,
        std::vector<Trajectory> &trajectories,
        bool finished
    ) :
        measurements(PyMeasurementVectorWrapper(measurements)),
        trajectories(PyTrajectoryVectorWrapper(trajectories)),
        finished(finished)
    {}
    virtual ~PyHeliosOutputWrapper() {}
};

}

#endif