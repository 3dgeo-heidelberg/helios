#pragma once

#include <string>
#include <unordered_map>

#include <Measurement.h>
#include <Scanner.h>
#include <ScannerSettings.h>
#include <adt/exprtree/UnivarExprTreeNode.h>
#include <util/FullWaveformYielder.h>
#include <util/PointcloudYielder.h>
#include <util/PulseRecordYielder.h>

namespace helios {
namespace filems {
class FMSFacade;
}
}
using helios::filems::FMSFacade;

/**
 * @brief Base abstract class for detectors
 */
class AbstractDetector
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Scanner which the detector belongs to
   */
  std::shared_ptr<Scanner> scanner = nullptr;

protected:
  /**
   * @brief Main facade to file management system
   */
  std::shared_ptr<FMSFacade> fms = nullptr;

public:
  /**
   * @brief The point cloud yielder which handles point cloud building from
   *  measurements
   */
  std::shared_ptr<PointcloudYielder> pcloudYielder = nullptr;
  /**
   * @brief The full waveform yielder which handles full waveform output
   */
  std::shared_ptr<FullWaveformYielder> fwfYielder = nullptr;
  /**
   * @brief The pulse record yielder which handles the pulse records output
   */
  std::shared_ptr<PulseRecordYielder> pulseRecordYielder = nullptr;

  /**
   * @brief Detector accuracy in meters
   */
  double cfg_device_accuracy_m = 0;
  /**
   * @brief Minimum range for detector in meters
   */
  double cfg_device_rangeMin_m = 0;
  /**
   * @brief Maximum range for detector in meters
   */
  double cfg_device_rangeMax_m;

  std::shared_ptr<UnivarExprTreeNode<double>> errorDistanceExpr = nullptr;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Base constructor for abstract detector
   * @see AbstractDetector::scanner
   * @see AbstractDetector::accuracy_m
   * @see AbstractDetector::rangeMin_m
   */
  AbstractDetector(
    std::shared_ptr<Scanner> scanner,
    double accuracy_m,
    double rangeMin_m,
    double rangeMax_m = std::numeric_limits<double>::max(),
    std::shared_ptr<UnivarExprTreeNode<double>> errorDistanceExpr = nullptr)
  {
    this->cfg_device_accuracy_m = accuracy_m;
    this->cfg_device_rangeMin_m = rangeMin_m;
    this->cfg_device_rangeMax_m = rangeMax_m;
    this->scanner = std::move(scanner);
    this->errorDistanceExpr = errorDistanceExpr;
  }
  virtual ~AbstractDetector() {}
  virtual std::shared_ptr<AbstractDetector> clone() = 0;
  virtual void _clone(std::shared_ptr<AbstractDetector> ad);

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Shutdown the detector when simulation has finished.
   */
  virtual void shutdown();
  /**
   * @brief Apply scanner settings to the detector
   * @param settings Settings to be applied to de detector
   */
  virtual void applySettings(std::shared_ptr<ScannerSettings>& settings) {};
  /**
   * @brief Handle detector behavior when leg has been completed.
   *
   * It mainly forces yielded point cloud to be flushed
   */
  virtual void onLegComplete();

  /**
   * @brief Check whether the given distance is inside detected range or not
   * @param distance The distance to be checked (in meters)
   * @return True if given distance is inside detected range, false otherwise
   */
  inline bool isDistanceInRange(double const distance)
  {
    return cfg_device_rangeMin_m <= distance &&
           cfg_device_rangeMax_m >= distance;
  }
  /**
   * @brief Check whether the given distance is inside detected range or not
   * @param distance The distance to be checked (in meters)
   * @return True if given distance is NOT inside detected range, false
   *  otherwise
   */
  inline bool isDistanceNotInRange(double const distance)
  {
    return cfg_device_rangeMin_m > distance || cfg_device_rangeMax_m < distance;
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the main facade to file management system
   * @return The main facade to file management system
   */
  inline std::shared_ptr<FMSFacade> getFMS() const { return fms; }
  /**
   * @brief Set the main facade to file management system that will be used
   *  by the detector
   * @param fms The new main facade to file management system for the
   *  detector
   */
  void setFMS(std::shared_ptr<FMSFacade> fms);
};
