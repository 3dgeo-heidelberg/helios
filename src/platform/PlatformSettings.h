#pragma once

#include <Asset.h>

#include <glm/glm.hpp>

#include <functional>
#include <memory>
#include <sstream>
#include <unordered_set>

/**
 * @brief Class representing platform settings
 */
class PlatformSettings : public Asset
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The ID for this platform settings.
   * It does not make sense for all platform settings, but it is specially
   * useful when it comes to handling XML templates
   */
  std::string id = "#nullid#";
  /**
   * @brief Template defining default values which were used to build the
   *  PlatformSettings object
   */
  std::shared_ptr<PlatformSettings> baseTemplate = nullptr;
  /**
   * @brief Position x coordinate
   */
  double x = 0;
  /**
   * @brief Position y coordinate
   */
  double y = 0;
  /**
   * @brief Position z coordinate
   */
  double z = 0;
  /**
   * @brief Flag
   */
  bool yawAtDepartureSpecified = false;
  /**
   * @brief Yaw angle (in radians) at platform departure
   */
  double yawAtDeparture = 0.0;

  /**
   * @brief On ground flag
   * @see Platform::onGround
   */
  bool onGround = false;
  /**
   * @brief Stop and turn flag
   * @see Platform::stopAndTurn
   */
  bool stopAndTurn = true;
  /**
   * @brief Smooth turn
   * @see Platform::smoothTurn
   */
  bool smoothTurn = false;
  /**
   * @brief Slowdown enabled flag
   * @see Platform::slowdownEnabled
   */
  bool slowdownEnabled = true;

  // 100 meter per sec are 360 km/h:
  /**
   * @brief Movement per seconds (in meters)
   */
  double movePerSec_m = 70;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Platform settings default constructor
   */
  PlatformSettings() = default;

  /**
   * @brief Copy from pointer constructor
   * @param other Platform settings to be copied
   */
  PlatformSettings(PlatformSettings* other)
  {
    if (other == nullptr)
      return;

    this->id = other->id;
    this->baseTemplate = other->baseTemplate;
    this->x = other->x;
    this->y = other->y;
    this->z = other->z;
    this->yawAtDepartureSpecified = other->yawAtDepartureSpecified;
    this->yawAtDeparture = other->yawAtDeparture;
    this->onGround = other->onGround;
    this->stopAndTurn = other->stopAndTurn;
    this->smoothTurn = other->smoothTurn;
    this->slowdownEnabled = other->slowdownEnabled;
    this->movePerSec_m = other->movePerSec_m;
  }

  // ***  CHERRY PICKING  *** //
  // ************************ //
  /**
   * @brief Build a new platform settings which by default has the same
   *  values than caller platform settings (this). Any field specified
   *  through fields set will be overloaded from cherries platform settings.
   * @param cherries From where overloaded values are taken
   * @param[in] fields Which fields must be overloaded
   * @param[in] templateFields Which fields must be overloaded for the
   *  template. Notice it can be nullptr in case there is no cherry template
   * @return New platform settings from cherry picking
   */
  std::shared_ptr<PlatformSettings> cherryPick(
    std::shared_ptr<PlatformSettings> cherries,
    std::unordered_set<std::string> const& fields,
    std::unordered_set<std::string> const* templateFields = nullptr)
  {
    // Prepare for cherry picking
    std::shared_ptr<PlatformSettings> settings =
      std::make_shared<PlatformSettings>(this);
    std::function<bool(std::string const&)> hasCherry =
      [&](std::string const& fieldName) -> bool {
      return fields.find(fieldName) != fields.end();
    };
    // The cherry picking itself
    if (hasCherry("baseTemplate") && cherries->baseTemplate != nullptr) {
      std::shared_ptr<PlatformSettings> tmpTemplate =
        cherries->baseTemplate->baseTemplate;
      cherries->baseTemplate->baseTemplate = nullptr;
      settings = cherryPick(cherries->baseTemplate, *templateFields);
      cherries->baseTemplate->baseTemplate = tmpTemplate;
      settings->baseTemplate = cherries->baseTemplate;
    }
    if (hasCherry("x"))
      settings->x = cherries->x;
    if (hasCherry("y"))
      settings->y = cherries->y;
    if (hasCherry("z"))
      settings->z = cherries->z;
    if (hasCherry("yawAtDepartureSpecified"))
      settings->yawAtDepartureSpecified = cherries->yawAtDepartureSpecified;
    if (hasCherry("yawAtDeparture"))
      settings->yawAtDeparture = cherries->yawAtDeparture;
    if (hasCherry("onGround"))
      settings->onGround = cherries->onGround;
    if (hasCherry("stopAndTurn"))
      settings->stopAndTurn = cherries->stopAndTurn;
    if (hasCherry("smoothTurn"))
      settings->smoothTurn = cherries->smoothTurn;
    if (hasCherry("slowdownEnabled"))
      settings->slowdownEnabled = cherries->slowdownEnabled;
    if (hasCherry("movePerSec_m"))
      settings->movePerSec_m = cherries->movePerSec_m;
    // Return settings from cherry picking
    return settings;
  }

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain position as 3D vector
   * @return Position as 3D vector
   */
  glm::dvec3 getPosition() { return glm::dvec3(x, y, z); }

  /**
   * @brief Set position from 3D vector
   * @param dest Position as 3D vector
   */
  void setPosition(glm::dvec3 dest)
  {
    x = dest.x;
    y = dest.y;
    z = dest.z;
  }
  void setPosition(double const x, double const y, double const z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  /**
   * @brief Check if this PlatformSettings has an associated template (true)
   *  or not (false)
   * @return True when there is an associated template, false otherwise
   * @see PlatformSettings::baseTemplate
   * @see PlatformSettings::hasTemplate
   */
  bool hasTemplate() { return this->baseTemplate != nullptr; }
  /**
   * @brief Obtain template by reference
   * @return Reference to template associated to this PlatformSettings
   * @see PlatformSettings::baseTemplate
   * @see PlatformSettings::hasTemplate
   */
  PlatformSettings& getTemplate() { return *this->baseTemplate; }

  // ***  TO STRING  *** //
  // ******************* //
  /**
   * @brief Obtain the string representation of the scanner settings
   * @return String representing the scanner settings
   */
  virtual inline std::string toString() const
  {
    std::stringstream ss;
    ss << "PlatformSettings \"" << id << "\":\n";
    if (baseTemplate != nullptr) {
      ss << "\ttemplate.id = \"" << baseTemplate->id << "\"\n"
         << "\ttemplate.x = " << baseTemplate->x << "\n"
         << "\ttemplate.y = " << baseTemplate->y << "\n"
         << "\ttemplate.z = " << baseTemplate->z << "\n"
         << "\ttemplate.yawAtDepartureSpecified = "
         << baseTemplate->yawAtDepartureSpecified << "\n"
         << "\ttemplate.yawAtDeparture = " << baseTemplate->yawAtDeparture
         << "\n"
         << "\ttemplate.onGround = " << baseTemplate->onGround << "\n"
         << "\ttemplate.stopAndTurn = " << baseTemplate->stopAndTurn << "\n"
         << "\ttemplate.smoothTurn = " << baseTemplate->smoothTurn << "\n"
         << "\ttemplate.slowdownEnabled = " << baseTemplate->slowdownEnabled
         << "\n"
         << "\ttemplate.movePerSec_m = " << baseTemplate->movePerSec_m << "\n";
    }
    ss << "x = " << x << "\n"
       << "y = " << y << "\n"
       << "z = " << z << "\n"
       << "yawAtDepartureSpecified = " << yawAtDepartureSpecified << "\n"
       << "yawAtDeparture = " << yawAtDeparture << "\n"
       << "onGround = " << onGround << "\n"
       << "stopAndTurn = " << stopAndTurn << "\n"
       << "smoothTurn = " << smoothTurn << "\n"
       << "slowdownEnabled = " << slowdownEnabled << "\n"
       << "movePerSec_m = " << movePerSec_m << "\n";
    return ss.str();
  }
  /**
   * @brief Overload of << operator for output streams
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  PlatformSettings const& settings)
  {
    out << settings.toString();
    return out;
  }
};
