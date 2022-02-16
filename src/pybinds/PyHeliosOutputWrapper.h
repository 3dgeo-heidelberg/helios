#pragma once

#ifdef PYTHON_BINDING

#include <PyMeasurementVectorWrapper.h>
#include <PyTrajectoryVectorWrapper.h>
#include <PyStringVector.h>

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
    std::string outpath;
    PyStringVector outpaths;
    bool finished;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyHeliosOutputWrapper(
        std::shared_ptr<std::vector<Measurement>> measurements,
        std::shared_ptr<std::vector<Trajectory>> trajectories,
        std::string const &outpath,
        std::shared_ptr<std::vector<std::string>> outpaths,
        bool finished
    ) :
        measurements(PyMeasurementVectorWrapper(*measurements)),
        trajectories(PyTrajectoryVectorWrapper(*trajectories)),
        outpath(outpath),
        outpaths(PyStringVector(*outpaths)),
        finished(finished)
    {}
    PyHeliosOutputWrapper(
        std::vector<Measurement> &measurements,
        std::vector<Trajectory> &trajectories,
        std::string const &outpath,
        std::vector<std::string> outpaths,
        bool finished
    ) :
        measurements(PyMeasurementVectorWrapper(measurements)),
        trajectories(PyTrajectoryVectorWrapper(trajectories)),
        outpath(outpath),
        outpaths(PyStringVector(outpaths)),
        finished(finished)
    {}
    virtual ~PyHeliosOutputWrapper() {}
};

}

#endif