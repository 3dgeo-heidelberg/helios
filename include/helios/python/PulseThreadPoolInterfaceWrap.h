#include <helios/scanner/detector/PulseThreadPoolInterface.h>

class PulseThreadPoolInterfaceWrap : public PulseThreadPoolInterface
{
public:
  using PulseThreadPoolInterface::PulseThreadPoolInterface; // Inherit
                                                            // constructors

  void run_pulse_task(TaskDropper<PulseTask,
                                  PulseThreadPoolInterface,
                                  std::vector<std::vector<double>>&,
                                  RandomnessGenerator<double>&,
                                  RandomnessGenerator<double>&,
                                  NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                                  ,
                                  std::shared_ptr<HDA_PulseRecorder>
#endif
                                  >& dropper) override
  {
    PYBIND11_OVERLOAD_PURE(void,                     // Return type
                           PulseThreadPoolInterface, // Parent class
                           run_pulse_task, // Name of function in Python
                           dropper         // Arguments
    );
  }

  bool try_run_pulse_task(TaskDropper<PulseTask,
                                      PulseThreadPoolInterface,
                                      std::vector<std::vector<double>>&,
                                      RandomnessGenerator<double>&,
                                      RandomnessGenerator<double>&,
                                      NoiseSource<double>&
#if DATA_ANALYTICS >= 2
                                      ,
                                      std::shared_ptr<HDA_PulseRecorder>
#endif
                                      >& dropper) override
  {
    PYBIND11_OVERLOAD_PURE(bool,                     // Return type
                           PulseThreadPoolInterface, // Parent class
                           try_run_pulse_task, // Name of function in Python
                           dropper             // Arguments
    );
  }

  void join() override
  {
    PYBIND11_OVERLOAD_PURE(void,                     // Return type
                           PulseThreadPoolInterface, // Parent class
                           join, // Name of function in Python
    );
  }
};
