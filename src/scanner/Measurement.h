#pragma once

#include <ostream>
#include <string>

#include <glm/glm.hpp>

#include <util/PrintUtils.h>

/**
 * @brief Class representing a measurement
 */
class Measurement
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Identifier of the device which generated the measurement
   */
  std::string devId;
  /**
   * @brief The index of the scanning device which generated the measurement
   */
  std::size_t devIdx;
  /**
   * @brief ID of hit object
   */
  std::string hitObjectId;
  /**
   * @biref Point position
   */
  glm::dvec3 position = glm::dvec3(0, 0, 0);
  /**
   * @brief Beam director vector
   */
  glm::dvec3 beamDirection = glm::dvec3(0, 0, 0);
  /**
   * @brief Beam origin
   */
  glm::dvec3 beamOrigin = glm::dvec3(0, 0, 0);
  /**
   * @brief Intersection distance
   */
  double distance = 0;
  /**
   * @brief Point intensity
   */
  double intensity = 0;
  /**
   * @brief Echo width
   */
  double echo_width = 0;
  /**
   * @brief Measurement return number
   */
  int returnNumber = 0;
  /**
   * @brief Pulse return number
   */
  int pulseReturnNumber = 0;
  /**
   * @brief Full wave index
   */
  int fullwaveIndex = 0;
  /**
   * @brief Point class
   */
  int classification = 0;
  /**
   * @brief Measurement GPS time
   */
  double gpsTime;
  /**
   * @brief Current leg index (only for point cloud output in Python API)
   */
  uint16_t currentLegIndex = 65535; // Default to 65535 indicating undefined

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default measurement constructor
   */
  Measurement() = default;
  Measurement(const Measurement& m)
  {
    devId = m.devId;
    devIdx = m.devIdx;
    hitObjectId = m.hitObjectId;
    position = glm::dvec3(m.position);
    beamDirection = glm::dvec3(m.beamDirection);
    beamOrigin = glm::dvec3(m.beamOrigin);
    distance = m.distance;
    intensity = m.intensity;
    echo_width = m.echo_width;
    returnNumber = m.returnNumber;
    pulseReturnNumber = m.pulseReturnNumber;
    fullwaveIndex = m.fullwaveIndex;
    classification = m.classification;
    gpsTime = m.gpsTime;
    currentLegIndex = m.currentLegIndex;
  }
  virtual ~Measurement() = default;

  // ***  O P E R A T O R S  *** //
  // *************************** //
  friend std::ostream& operator<<(std::ostream& out, Measurement& m)
  {
    out << m.hitObjectId << "," << m.position << "," << m.beamDirection << ","
        << m.beamOrigin << "," << m.distance << "," << m.intensity << ","
        << m.echo_width << "," << m.returnNumber << "," << m.pulseReturnNumber
        << "," << m.fullwaveIndex << "," << m.classification << ","
        << m.gpsTime;
    return out;
  }
};
