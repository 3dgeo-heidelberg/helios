#include <helios/sim/comps/SimulationReporter.h>
#include <helios/sim/core/Simulation.h>
#include <helios/util/logger/logging.hpp>

#include <memory>
#include <sstream>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SimulationReporter::SimulationReporter(Simulation const& sim)
  : sim(sim)
{
}

// ***  REPORT METHODS  *** //
// ************************ //
void
SimulationReporter::preStartReport() const
{
  std::stringstream ss;
  ss << "SIMULATION START\n";
  std::string dmoReport = reportDynMovingObjects();
  if (dmoReport != "")
    ss << dmoReport;
  logging::DEBUG(ss.str());
}

void
SimulationReporter::preFinishReport(double const seconds) const
{
  std::stringstream ss;

  // Report dynamic objects
  ss << "SIMULATION LOOP COMPLETED\n";
  std::string dmoReport = reportDynMovingObjects();
  if (dmoReport != "")
    ss << dmoReport;
  logging::DEBUG(ss.str());

  // Report steps and times
  ss.str("");
  ss << "Elapsed simulation steps = " << sim.stepLoop.getCurrentStep() << "\n"
     << "Elapsed virtual time = " << sim.stepLoop.getCurrentTime() << " sec.\n"
     << "Main thread simulation loop finished in " << seconds << " sec.\n"
     << "Waiting for completion of pulse computation tasks...";
  logging::TIME(ss.str());
}

void
SimulationReporter::postFinishReport(double const seconds) const
{
  // Report computation task including finish process
  std::stringstream ss;
  ss << "Pulse computation tasks finished in " << seconds << " sec.";
  logging::TIME(ss.str());
}

// ***  UTIL METHODS  *** //
// ********************** //
std::string
SimulationReporter::reportDynMovingObjects() const
{
  // Check that there are moving objects
  Scene& scene = *(sim.mScanner->platform->scene);
  if (!scene.hasMovingObjects())
    return "";

  // Generate string report for moving objects
  std::stringstream ss;
  ss << "Dynamic moving objects:\n";
  for (std::shared_ptr<ScenePart> sp : scene.parts) {
    if (sp->getType() != ScenePart::DYN_MOVING_OBJECT)
      continue;
    DynMovingObject* dmo = (DynMovingObject*)sp.get();
    arma::colvec& c = dmo->centroid;
    ss << dmo->getId() << "\n\tcentroid: (" << c[0] << ", " << c[1] << ", "
       << c[2] << ")\n";
  }
  return ss.str();
}
