#include <cmath>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <string>

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
    py_obj(measurements, trajectories, outpath);
  }

private:
  py::object py_obj;
};
