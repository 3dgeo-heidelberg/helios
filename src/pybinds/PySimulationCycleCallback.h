#pragma once

#include <PyHeliosOutputWrapper.h>
#include <SimulationCycleCallback.h>
#include <boost/python.hpp>

using boost::ref;
using boost::python::call;

namespace pyhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Python callback for each simulation cycle that has been completed
 *
 * @see PyHeliosOutputWrapper
 */
class PySimulationCycleCallback : public SimulationCycleCallback
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  PyObject* pyCallback;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  PySimulationCycleCallback(PyObject* pyCallback)
    : pyCallback(pyCallback)
  {
  }
  ~PySimulationCycleCallback() override {}

  // ***  F U N C T O R  *** //
  // *********************** //
  void operator()(std::vector<Measurement>& measurements,
                  std::vector<Trajectory>& trajectories,
                  std::string const& outpath) override
  {
    PyHeliosOutputWrapper phow(measurements,
                               trajectories,
                               outpath,
                               std::vector<std::string>{ outpath },
                               false);
    PyGILState_STATE gilState = PyGILState_Ensure();
    call<void>(pyCallback, ref(phow));
    PyGILState_Release(gilState);
  }
};

}
