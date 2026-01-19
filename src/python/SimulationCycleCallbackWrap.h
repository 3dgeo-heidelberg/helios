#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <string>
#include <cmath>

namespace py = pybind11;

class [[gnu::visibility("default")]] SimulationCycleCallbackWrap : public SimulationCycleCallback {
public:
    using SimulationCycleCallback::SimulationCycleCallback;
    SimulationCycleCallbackWrap(py::object obj) : SimulationCycleCallback(), py_obj(std::move(obj)) {}

    void operator()(
        std::vector<Measurement> &measurements,
        std::vector<Trajectory> &trajectories,
        const std::string &outpath) override {

        py::gil_scoped_acquire acquire;  // Acquire GIL before calling Python code

        // Create a tuple with the required components
        py::tuple output = py::make_tuple(
            measurements,                       
            trajectories,                       
            outpath,                            
            std::vector<std::string>{outpath},  
            false                               
        );

        
        py_obj(output);
    }

private:
    py::object py_obj;

};
