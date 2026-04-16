#pragma once

#include "AbstractBeamDeflector.h"
/**
 * @brief Class representing a conic beam deflector
 */
class ConicBeamDeflector : public AbstractBeamDeflector
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** /7
  /**
   * @brief Rotation used to create the radius of the cone
   */
  Rotation r1;
  /**
   * @brief Semi-major axis angle in radians (across-track)
   */
  double acrossTrackAngle;

  /**
   * @brief Semi-minor axis angle in radians (along-track)
   */
  double alongTrackAngle;
  /**
   * @brief Flag indicating whether to use elliptical pattern (true) or circular
   * (false)
   */
  bool ellipticalMode;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for conic beam deflector
   * @see AbstractBeamDeflector::AbstractBeamDeflector(
   *  double, double, double)
   */
  ConicBeamDeflector(double scanAngleMax_rad,
                     double scanFreqMax_Hz,
                     double scanFreqMin_Hz)
    : AbstractBeamDeflector(scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz)
    , acrossTrackAngle(scanAngleMax_rad) // Default to circular
    , alongTrackAngle(scanAngleMax_rad)  // Default to circular
    , ellipticalMode(false)              // Default to circular
  {
  }

  /**
   * @brief Constructor for elliptical conic beam deflector
   */
  ConicBeamDeflector(double scanAngleMax_rad,
                     double scanFreqMax_Hz,
                     double scanFreqMin_Hz,
                     double acrossTrackAngle_rad,
                     double alongTrackAngle_rad)
    : AbstractBeamDeflector(scanAngleMax_rad, scanFreqMax_Hz, scanFreqMin_Hz)
    , acrossTrackAngle(acrossTrackAngle_rad)
    , alongTrackAngle(alongTrackAngle_rad)
    , ellipticalMode(true)
  {
    if (!std::isfinite(acrossTrackAngle_rad) ||
        !std::isfinite(alongTrackAngle_rad) || acrossTrackAngle_rad < 0.0 ||
        alongTrackAngle_rad < 0.0) {
      std::stringstream ss;
      ss << "ConicBeamDeflector::ConicBeamDeflector(double, double, double, "
         << "double, double) received invalid elliptical cone angles.";
      throw std::invalid_argument(ss.str());
    }
  }

  std::shared_ptr<AbstractBeamDeflector> clone() override;
  void _clone(std::shared_ptr<AbstractBeamDeflector> abd) override;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @see AbstractBeamDeflector::applySettings
   */
  void applySettings(std::shared_ptr<ScannerSettings> settings) override;
  /**
   * @see AbstractBeamDeflector::doSimStep
   */
  void doSimStep() override;

  /**
   * @see AbstractBeamDeflector::getOpticsType
   */
  std::string getOpticsType() const override
  {
    return ellipticalMode ? "ELLIPTICAL_CONIC" : "CONIC";
  }
};
