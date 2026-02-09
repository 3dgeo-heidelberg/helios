#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "fluxionum/DesignMatrix.h"
#include "fluxionum/TemporalDesignMatrix.h"
#include <armadillo>
#include <memory>
#include <platform/InterpolatedMovingPlatformEgg.h>
#include <platform/LinearPathPlatform.h>
#include <platform/Platform.h>

namespace py = pybind11;
using fluxionum::DesignMatrix;
using fluxionum::TemporalDesignMatrix;

// std::shared_ptr<InterpolatedMovingPlatformEgg>
std::shared_ptr<Platform>
load_interpolated_platform(std::shared_ptr<LinearPathPlatform> basePlatform,
                           py::array trajectory,
                           std::string rotspec,
                           bool syncGPSTime);
