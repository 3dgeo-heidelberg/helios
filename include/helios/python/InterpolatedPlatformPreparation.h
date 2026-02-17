#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <armadillo>
#include <helios/maths/fluxionum/DesignMatrix.h>
#include <helios/maths/fluxionum/TemporalDesignMatrix.h>
#include <helios/platform/InterpolatedMovingPlatformEgg.h>
#include <helios/platform/LinearPathPlatform.h>
#include <helios/platform/Platform.h>
#include <memory>

namespace py = pybind11;
using fluxionum::DesignMatrix;
using fluxionum::TemporalDesignMatrix;

// std::shared_ptr<InterpolatedMovingPlatformEgg>
std::shared_ptr<Platform>
load_interpolated_platform(std::shared_ptr<LinearPathPlatform> basePlatform,
                           py::array trajectory,
                           std::string rotspec,
                           bool syncGPSTime);
