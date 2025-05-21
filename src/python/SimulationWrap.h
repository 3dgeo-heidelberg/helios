#include <pybind11/pybind11.h>
#include <sim/core/Simulation.h>

class SimulationWrap : public Simulation
{
public:
  using Simulation::Simulation;

  void onLegComplete() override
  {
    PYBIND11_OVERRIDE_PURE(void, Simulation, onLegComplete, );
  }

  void doSimLoop() override
  {
    PYBIND11_OVERRIDE(void, Simulation, doSimLoop, );
  }

  void shutdown() override { PYBIND11_OVERRIDE(void, Simulation, shutdown, ); }
};
