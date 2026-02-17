#include <helios/scanner/ScanningPulseProcess.h>

class ScanningPulseProcessWrap : public ScanningPulseProcess
{
public:
  using ScanningPulseProcess::ScanningPulseProcess; // Inherit constructors

  // Override handlePulseComputation in Python
  void handlePulseComputation(SimulatedPulse const& sp) override
  {
    PYBIND11_OVERRIDE_PURE(void,                   // Return type
                           ScanningPulseProcess,   // Parent class
                           handlePulseComputation, // Function name
                           sp                      // Argument(s)
    );
  }

  // Override onLegComplete in Python
  void onLegComplete() override
  {
    PYBIND11_OVERRIDE(void,                 // Return type
                      ScanningPulseProcess, // Parent class
                      onLegComplete         // Function name
    );
  }

  // Override onSimulationFinished in Python
  void onSimulationFinished() override
  {
    PYBIND11_OVERRIDE(void,                 // Return type
                      ScanningPulseProcess, // Parent class
                      onSimulationFinished  // Function name
    );
  }
};
