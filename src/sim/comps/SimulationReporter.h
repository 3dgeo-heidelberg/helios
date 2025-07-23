#pragma once

class Simulation;

#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle reports related to simulation
 * @see Simulation
 */
class SimulationReporter
{
protected:
  /**
   * @brief The simulation to report about
   */
  Simulation const& sim;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  SimulationReporter(Simulation const& sim);

  // ***  REPORT METHODS  *** //
  // ************************ //
  /**
   * @brief Report what must be reported immediately before starting the
   *  simulation
   */
  void preStartReport() const;
  /**
   * @brief Report what must be reported immediately before finishing the
   *  simulation
   * @param[in] seconds How many seconds elapsed until finish process
   *  started
   */
  void preFinishReport(double const seconds) const;
  /**
   * @brief Report what must be reported immediately after finishing the
   *  simulation but before its shutdown
   * @param[in] seconds How many seconds elapsed until finish process
   *  was completed
   */
  void postFinishReport(double const seconds) const;

protected:
  // ***  UTIL METHODS  *** //
  // ********************** //
  /**
   * @brief Generate the string report of dynamic moving objects
   * @return Empty string if there are no moving objects in the scene.
   *  Otherwise, the report string of moving objects.
   */
  std::string reportDynMovingObjects() const;
};
