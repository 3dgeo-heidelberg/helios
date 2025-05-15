#pragma once

#include <PyHeliosUtils.h>
#include <PyTrajectoryWrapper.h>
#include <vector>

namespace pyhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Python wrapper for helios trajectory
 *
 * @see PyTrajectoryWrapper
 * @see PyHeliosOutputWrapper
 */
class PyTrajectoryVectorWrapper
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  std::vector<Trajectory> allTrajectories;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  PyTrajectoryVectorWrapper(std::vector<Trajectory>& allTrajectories)
    : allTrajectories(allTrajectories)
  {
  }
  virtual ~PyTrajectoryVectorWrapper() {}

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  PyTrajectoryWrapper* get(size_t index)
  {
    return new PyTrajectoryWrapper(
      allTrajectories[PyHeliosUtils::handlePythonIndex(
        index, allTrajectories.size())]);
  }
  void erase(size_t index)
  {
    allTrajectories.erase(
      allTrajectories.begin() +
      PyHeliosUtils::handlePythonIndex(index, allTrajectories.size()));
  }
  size_t length() { return allTrajectories.size(); }
};

}
