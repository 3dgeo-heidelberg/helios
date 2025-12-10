#pragma once

#include "Asset.h"

#include <cmath>
#include <functional>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_set>

/**
 * @brief Scanner settings class
 */
class ScannerSettings : public Asset
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The ID for this scanner settings.
   * It does not make sense for all scanner settings, but it is specially
   * useful when it comes to handling XML templates
   */
  std::string id = "#nullid#";
  /**
   * @brief Template defining default values which were used to build the
   *  ScannerSettings object
   */
  std::shared_ptr<ScannerSettings> baseTemplate = nullptr;

  /**
   * @brief Flag to specify if scanner is active (true) or not (false)
   *
   * When a scanner is not active, no points will be captured
   */
  bool active = true;
  /**
   * @brief Amount of rotation (radians) per second for the scanner head
   */
  double headRotatePerSec_rad = 0;
  /**
   * @brief Starting angle (radians) for the scanner head
   */
  double headRotateStart_rad = 0;
  /**
   * @brief Ending angle (radians) for the scanner head
   */
  double headRotateStop_rad = 0;
  /**
   * @brief Pulse frequency (hertz)
   */
  int pulseFreq_Hz = 0;
  /**
   * @brief Scan angle (radians)
   */
  double scanAngle_rad = 0;
  /**
   * @brief Minimum vertical angle (radians)
   */
  double verticalAngleMin_rad = NAN;
  /**
   * @brief Maximum vertical angle (radians)
   */
  double verticalAngleMax_rad = NAN;
  /**
   * @brief Scanning frequency (hertz)
   */
  double scanFreq_Hz = 0;
  /**
   * @brief Beam divergence angle (radians)
   */
  double beamDivAngle = 0.003;
  /**
   * @brief Time interval between trajectory recollections (seconds)
   */
  double trajectoryTimeInterval = 0.0; // In seconds
  /**
   * @brief Optional maximum duration (seconds) for a leg; <=0 disables
   */
  double maxDuration_s = -1.0;
  /**
   * @brief Specify the vertical resolution to be used. By default,
   *  vertical and horizontal resolutions are \f$0\f$ which means they will
   *  be ignored. When at least one of them is distinct than \f$0\f$, the
   *  scanner frequency and the head rotate per sec will be calculated
   *  from the resolutions ignoring given values
   * @see ScannerSettings::scanFreq_hz
   * @see ScannerSettings::headRotatePerSec_rad
   * @see ScannerSettings::horizontalResolution_rad
   */
  double verticalResolution_rad = 0.0;
  /**
   * @brief Specify the horizontal resolution to be used.
   * @see ScannerSettings::verticalResolution_rad
   */
  double horizontalResolution_rad = 0.0;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Scanner settings default constructor
   */
  ScannerSettings() = default;

  /**
   * @brief Copy from pointer constructor
   * @param other Scanner settings to be copied
   */
  ScannerSettings(ScannerSettings* other)
  {
    if (other == nullptr)
      return;

    this->id = other->id;
    this->baseTemplate = other->baseTemplate;
    this->active = other->active;
    this->headRotatePerSec_rad = other->headRotatePerSec_rad;
    this->headRotateStart_rad = other->headRotateStart_rad;
    this->headRotateStop_rad = other->headRotateStop_rad;
    this->pulseFreq_Hz = other->pulseFreq_Hz;
    this->scanAngle_rad = other->scanAngle_rad;
    this->verticalAngleMin_rad = other->verticalAngleMin_rad;
    this->verticalAngleMax_rad = other->verticalAngleMax_rad;
    this->scanFreq_Hz = other->scanFreq_Hz;
    this->beamDivAngle = other->beamDivAngle;
    this->trajectoryTimeInterval = other->trajectoryTimeInterval;
    this->maxDuration_s = other->maxDuration_s;
    this->verticalResolution_rad = other->verticalResolution_rad;
    this->horizontalResolution_rad = other->horizontalResolution_rad;
  }

  // ***  CHERRY PICKING  *** //
  // ************************ //
  /**
   * @brief Build a new scanner settings which by default has the same values
   *  than caller scanner settings (this). Any field specified through fields
   *  set will be overloaded from cherries scanner settings.
   * @param cherries From where overloaded values are taken
   * @param[in] fields Which fields must be overloaded
   * @param[in] templateFields Which fields must be overloaded for the
   *  template. Notice it can be nullptr in case there is no cherry template
   * @return New scanner settings from cherry picking
   */
  std::shared_ptr<ScannerSettings> cherryPick(
    std::shared_ptr<ScannerSettings> cherries,
    std::unordered_set<std::string> const& fields,
    std::unordered_set<std::string> const* templateFields = nullptr)
  {
    // Prepare for cherry picking
    std::shared_ptr<ScannerSettings> settings =
      std::make_shared<ScannerSettings>(this);
    std::function<bool(std::string const&)> hasCherry =
      [&](std::string const& fieldName) -> bool {
      return fields.find(fieldName) != fields.end();
    };
    // The cherry picking itself
    if (hasCherry("baseTemplate") && cherries->baseTemplate != nullptr) {
      std::shared_ptr<ScannerSettings> tmpTemplate =
        cherries->baseTemplate->baseTemplate;
      cherries->baseTemplate->baseTemplate = nullptr;
      settings = cherryPick(cherries->baseTemplate, *templateFields);
      cherries->baseTemplate->baseTemplate = tmpTemplate;
      settings->baseTemplate = cherries->baseTemplate;
    }
    if (hasCherry("active"))
      settings->active = cherries->active;
    if (hasCherry("headRotatePerSec_rad"))
      settings->headRotatePerSec_rad = cherries->headRotatePerSec_rad;
    if (hasCherry("headRotateStart_rad"))
      settings->headRotateStart_rad = cherries->headRotateStart_rad;
    if (hasCherry("headRotateStop_rad"))
      settings->headRotateStop_rad = cherries->headRotateStop_rad;
    if (hasCherry("pulseFreq_Hz"))
      settings->pulseFreq_Hz = cherries->pulseFreq_Hz;
    if (hasCherry("scanAngle_rad"))
      settings->scanAngle_rad = cherries->scanAngle_rad;
    if (hasCherry("verticalAngleMin_rad"))
      settings->verticalAngleMin_rad = cherries->verticalAngleMin_rad;
    if (hasCherry("verticalAngleMax_rad"))
      settings->verticalAngleMax_rad = cherries->verticalAngleMax_rad;
    if (hasCherry("scanFreq_Hz"))
      settings->scanFreq_Hz = cherries->scanFreq_Hz;
    if (hasCherry("beamDivAngle"))
      settings->beamDivAngle = cherries->beamDivAngle;
    if (hasCherry("trajectoryTimeInterval"))
      settings->trajectoryTimeInterval = cherries->trajectoryTimeInterval;
    if (hasCherry("maxDuration_s"))
      settings->maxDuration_s = cherries->maxDuration_s;
    if (hasCherry("verticalResolution_rad"))
      settings->verticalResolution_rad = cherries->verticalResolution_rad;
    if (hasCherry("horizontalResolution_rad")) {
      settings->horizontalResolution_rad = cherries->horizontalResolution_rad;
    }

    // Return settings from cherry picking
    return settings;
  }

  // ***  CONFIG METHODS  *** //
  // ************************ //
  /**
   * @brief Update the settings to fit the specified resolution.
   *
   * Let \f$V_{\mathrm{res}}\f$ be the given vertical resolution,
   *  \f$H_{\mathrm{res}}\f$ be the given horizontal resolution,
   *  \f$f_{p}\f$ be the given pulse frequency, and \f$\alpha^*\f$ be the
   *  max scan angle.
   *
   * But then, the scanning frequency \f$f_s\f$ can be determined as:
   * \f[
   *  f_s = \frac{V_{\mathrm{res}} f_p}{2 \alpha^*}
   * \f]
   *
   * Also, the head rotation per second \f$H_{\mathrm{rps}}\f$ can be
   *  determined as:
   * \f[
   *  H_{\mathrm{rps}} = H_{\mathrm{res}} f_s
   * \f]
   *
   * @param scanAngleMax_rad \f$\alpha^*\f$
   */
  inline void fitToResolution(double const scanAngleMax_rad)
  {
    scanFreq_Hz =
      (pulseFreq_Hz * verticalResolution_rad) / (2.0 * scanAngleMax_rad);
    headRotatePerSec_rad = horizontalResolution_rad * scanFreq_Hz;
  }

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Check if this ScannerSettings has an associated template (true)
   *  or not (false)
   * @return True when there is an associated template, false otherwise
   * @see ScannerSettings::baseTemplate
   * @see ScannerSettings::hasTemplate
   */
  bool hasTemplate() { return this->baseTemplate != nullptr; }
  /**
   * @brief Obtain template by reference
   * @return Reference to template associated to this ScannerSettings
   * @see ScannerSettings::baseTemplate
   * @see ScannerSettings::hasTemplate
   */
  ScannerSettings& getTemplate() { return *this->baseTemplate; }

  /**
   * @brief Check whether the scanner settings' vertical and horizontal
   *  resolutions have the default values (disabled) or not (enabled).
   * @return True if vertical and horizontal resolutions have default
   *  null values (both 0), False otherwise.
   */
  inline bool hasDefaultResolution()
  {
    return verticalResolution_rad == 0 && horizontalResolution_rad == 0;
  }

  // ***  TO STRING  *** //
  // ******************* //
  /**
   * @brief Obtain the string representation of the scanner settings
   * @return String representing the scanner settings
   */
  virtual inline std::string toString() const
  {
    std::stringstream ss;
    ss << "ScannerSettings \"" << id << "\":\n";
    if (baseTemplate != nullptr) {
      ss << "\ttemplate.id = \"" << baseTemplate->id << "\"\n"
         << "\ttemplate.active = " << baseTemplate->active << "\n"
         << "\ttemplate.headRotatePerSec_rad = "
         << baseTemplate->headRotatePerSec_rad << "\n"
         << "\ttemplate.headRotateStart_rad = "
         << baseTemplate->headRotateStart_rad << "\n"
         << "\ttemplate.headRotateStop_rad = "
         << baseTemplate->headRotateStop_rad << "\n"
         << "\ttemplate.pulseFreq_Hz = " << baseTemplate->pulseFreq_Hz << "\n"
         << "\ttemplate.scanAngle_rad = " << baseTemplate->scanAngle_rad << "\n"
         << "\ttemplate.verticalAngleMin_rad = "
         << baseTemplate->verticalAngleMin_rad << "\n"
         << "\ttemplate.verticalAngleMax_rad = "
         << baseTemplate->verticalAngleMax_rad << "\n"
         << "\ttemplate.scanFreq_Hz = " << baseTemplate->scanFreq_Hz << "\n"
         << "\ttemplate.beamDivAngle = " << baseTemplate->beamDivAngle << "\n"
         << "\ttemplate.trajectoryTimeInterval = "
         << baseTemplate->trajectoryTimeInterval << "\n";
    }
    ss << "active = " << active << "\n"
       << "headRotatePerSec_rad = " << headRotatePerSec_rad << "\n"
       << "headRotateStart_rad = " << headRotateStart_rad << "\n"
       << "headRotateStop_rad = " << headRotateStop_rad << "\n"
       << "pulseFreq_Hz = " << pulseFreq_Hz << "\n"
       << "scanAngle_rad = " << scanAngle_rad << "\n"
       << "verticalAngleMin_rad = " << verticalAngleMin_rad << "\n"
       << "verticalAngleMax_rad = " << verticalAngleMax_rad << "\n"
       << "scanFreq_Hz = " << scanFreq_Hz << "\n"
       << "beamDivAngle = " << beamDivAngle << "\n"
       << "trajectoryTimeInterval = " << trajectoryTimeInterval << "\n"
       << "verticalResolution_rad = " << verticalResolution_rad << "\n"
       << "horizontalResolution_rad = " << horizontalResolution_rad << "\n";
    return ss.str();
  }
  /**
   * @brief Overload of << operator for output streams
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  const ScannerSettings& settings)
  {
    out << settings.toString();
    return out;
  }
};
