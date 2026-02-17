#pragma once

#include <helios/sim/comps/SimulationCycleCallback.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;

class [[gnu::visibility("default")]] SimulationCycleCallbackWrap
  : public SimulationCycleCallback
{
public:
  using SimulationCycleCallback::SimulationCycleCallback;
  SimulationCycleCallbackWrap(py::object obj)
    : SimulationCycleCallback()
    , py_obj(std::move(obj))
  {
  }

  void operator()(std::vector<Measurement>& measurements,
                  std::vector<Trajectory>& trajectories,
                  const std::string& outpath) override
  {

    py::gil_scoped_acquire acquire; // Acquire GIL before calling Python code

    // Create a tuple with the required components
    py::tuple output = py::make_tuple(measurements,
                                      trajectories,
                                      outpath,
                                      std::vector<std::string>{ outpath },
                                      false);

    py_obj(output);
  }

private:
  py::object py_obj;
};

class PySimulationCycleCallback : public SimulationCycleCallback
{
public:
  using SimulationCycleCallback::SimulationCycleCallback;

  bool is_callback_in_progress = false; // Flag to prevent recursion

  void operator()(std::vector<Measurement>& measurements,
                  std::vector<Trajectory>& trajectories,
                  const std::string& outpath) override
  {

    if (is_callback_in_progress) {
      return;
    }

    is_callback_in_progress = true; // Set the flag to prevent recursion

    py::gil_scoped_acquire acquire; // Acquire GIL before calling Python code

    py::object py_self = py::cast(
      this,
      py::return_value_policy::reference); // Reference to the Python object

    if (py::hasattr(py_self, "__call__")) {
      // Convert C++ vectors to Python lists
      py::list measurements_list = py::cast(measurements);
      py::list trajectories_list = py::cast(trajectories);

      // Call the Python __call__ method with converted arguments
      py_self.attr("__call__")(measurements_list, trajectories_list, outpath);
    } else {
      throw std::runtime_error(
        "Python __call__ method is missing on SimulationCycleCallback.");
    }

    is_callback_in_progress = false; // Reset the flag after callback completes
  }
};
