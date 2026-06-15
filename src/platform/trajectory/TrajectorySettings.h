#pragma once

#include <Asset.h>

#include <limits>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class representing trajectory settings
 */
class TrajectorySettings : public Asset
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The start time \f$t_a\f$ of the trajectory
   *
   * For instance, when the trajectory is interpolated from given data, the
   *  start position of that leg will be the one resulting from evaluating
   *  the interpolation function at given start time \f$t_a\f$
   *
   * <b>NOTICE</b> a value of std::numeric_limits<double>::lowest means the
   *  start time is the first known time
   */
  double tStart;
  /**
   * @brief The end time \f$t_b\f$ of the trajectory
   *
   * For instance, when the trajectory is interpolated from given data, the
   *  end position of that leg will be the one resulting from evaluating
   *  the interpolation function at given end time \f$t_b\f$
   *
   * <b>NOTICE</b> a value of std::numeric_limits<double>::max means the
   *  end time is the last known time
   */
  double tEnd;
  /**
   * @brief When true, the platform will be teleported to the start point
   *  of the next leg's trajectory when starting the leg. When false
   *  (default), the platform will continue from its last position.
   */
  bool teleportToStart;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Trajectory settings default constructor
   */
  TrajectorySettings()
    : tStart(std::numeric_limits<double>::lowest())
    , tEnd(std::numeric_limits<double>::max())
    , teleportToStart(false)
  {
  }
  ~TrajectorySettings() override = default;

  // ***  GETTERs and SETTErs  *** //
  // ***************************** //
  /**
   * @brief Check whether the start time (tStart) \f$t_a\f$ of this
   *  TrajectorySettings is setted or not
   * @return True if the start time is setted, false otherwise
   * @see TrajectorySettings::tStart
   */
  inline bool hasStartTime() const
  {
    return tStart != std::numeric_limits<double>::lowest();
  }
  /**
   * @brief Check whether the end time (tEnd) \f$t_b\f$ of this
   *  TrajectorySettings is setted or not
   * @return True if the end time is setted, false otherwise
   * @see TrajectorySettings::tEnd
   */
  inline bool hasEndTime() const
  {
    return tEnd != std::numeric_limits<double>::max();
  }
};
