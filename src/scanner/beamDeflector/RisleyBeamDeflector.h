#pragma once

#include <string>

#include "AbstractBeamDeflector.h"

#include "maths/Directions.h"
#include "maths/MathConverter.h"

#include <glm/glm.hpp>

#include <vector>

/**
 * @brief Struct representing a prism in the Risley beam deflector
 * This struct encapsulates the properties of a prism, including its angle,
 * refractive index, and rotation speed.
 */
struct Prism
{
  double angle_rad;     // Inclination angle
  double sin_angle_rad; // Sine of the inclination angle
  double cos_angle_rad; // Cosine of the inclination angle
  bool inclinedOnLeft;  // true: left side inclined, false: right side inclined
  double refractive_index;   // Refraction index of prism
  double rotation_speed_rad; // Angular velocity around z

  /**
   * @brief Get the surface normal vector for the first surface of the prism
   * @param time The time at which to evaluate the surface normal
   * @return The surface normal vector
   */
  glm::dvec3 getSurfaceNormal1(double time) const
  {
    double angle = rotation_speed_rad * time;
    if (inclinedOnLeft) {
      return glm::normalize(glm::dvec3(sin(angle) * sin_angle_rad,
                                       -cos_angle_rad,
                                       cos(angle) * sin_angle_rad));
    } else {
      return -Directions::forward;
    }
  }

  /**
   * @brief Get the surface normal vector for the second surface of the prism
   * @param time The time at which to evaluate the surface normal
   * @return The surface normal vector
   */
  glm::dvec3 getSurfaceNormal2(double time) const
  {
    double angle = rotation_speed_rad * time;
    if (inclinedOnLeft) {
      return -Directions::forward;
    } else {
      return glm::normalize(glm::dvec3(-sin(angle) * sin_angle_rad,
                                       -cos_angle_rad,
                                       -cos(angle) * sin_angle_rad));
    }
  }

  /**
   * @brief Computes the refracted beam direction using Snell's law.
   *
   * Given an incident beam direction and a surface normal, this function
   * calculates the direction of the refracted beam at the interface between
   * two media with different refractive indices.
   *
   * @param incidentBeamDirection Incident beam direction (unit vector,
   * glm::dvec3).
   * @param surfaceNormal Surface normal vector pointing out of the interface
   * (unit vector, glm::dvec3).
   * @param refractiveIdxA Refractive index of the medium the beam is coming
   * from.
   * @param refractiveIdxB Refractive index of the medium the beam is entering.
   * @param refracted Output parameter: the refracted beam direction
   * (glm::dvec3).
   *
   * @return true if refraction occurs, false if there is total internal
   * reflection.
   *
   * @note The input vectors `incidentBeamDirection` and `surfaceNormal` are
   * expected to be normalized.
   */
  bool refractBeam(const glm::dvec3& incidentBeamDirection,
                   const glm::dvec3& surfaceNormal,
                   double refractiveIdxA,
                   double refractiveIdxB,
                   glm::dvec3& refracted) const
  {
    double cos_delta_i = glm::dot(incidentBeamDirection, -surfaceNormal);
    double sin2_delta_r = (refractiveIdxA / refractiveIdxB) *
                          (refractiveIdxA / refractiveIdxB) *
                          (1.0 - cos_delta_i * cos_delta_i);
    if (sin2_delta_r > 1.0)
      return false; // total internal reflection

    double cos_delta_r = sqrt(1.0 - sin2_delta_r);
    refracted = glm::normalize(
      (refractiveIdxA / refractiveIdxB) * incidentBeamDirection +
      (refractiveIdxA / refractiveIdxB * cos_delta_i - cos_delta_r) *
        surfaceNormal);
    return true;
  }

  /**
   * @brief Refracts the beam through both surfaces of the prism.
   *
   * This function applies Snell's law to refract the incident beam direction
   * through both surfaces of the prism, returning the final refracted beam
   * direction.
   *
   * @param incidentBeamDirection The direction of the incident beam (unit
   * vector, glm::dvec3).
   * @param time The current simulation time.
   * @param refrIndex_air Refractive index of air.
   * @param refracted Output parameter: the final refracted beam direction
   * (glm::dvec3).
   *
   * @return true if refraction through both surfaces is successful, false if
   * there is total internal reflection at any surface.
   */
  bool refractPrism(const glm::dvec3& incidentBeamDirection,
                    double time,
                    double refrIndex_air,
                    glm::dvec3& refracted) const
  {
    // Refract through the first surface
    glm::dvec3 surfaceNormal1 = getSurfaceNormal1(time);
    glm::dvec3 afterFirst;
    if (!refractBeam(incidentBeamDirection,
                     surfaceNormal1,
                     refrIndex_air,
                     refractive_index,
                     afterFirst))
      return false;

    // Refract through the second surface
    glm::dvec3 surfaceNormal2 = getSurfaceNormal2(time);
    if (!refractBeam(afterFirst,
                     surfaceNormal2,
                     refractive_index,
                     refrIndex_air,
                     refracted))
      return false;

    return true;
  }
};

/**
 * @brief Class representing a Risley-prism beam deflector based on three
 * independently rotating prisms according to A. Li, X. Liu, and
 * W. Sun, "Forward and inverse solutions for three-element Risley prism
 * beam scanners," Opt. Express 25, 7677-7688 (2017), DOI: 10.1364/OE.25.007677,
 * or C. Qin, Y. Wang, W. Cai, H. Zhang, Z. Xiao, "Closed form analytical
 * solution and scan pattern shaping theory of three-element Risley prism for
 * LiDAR systems", Opt. Comm. 570, 130915 (2024),
 * DOI: 10.1016/j.optcom.2024.130915
 */
class RisleyBeamDeflector : public AbstractBeamDeflector
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //

  double deltaT = 0;
  double pulseFreq = 0;
  double time = 0;

  /**
   * @brief Refractive index of air
   */
  double refrIndex_air = 0;

  /**
   * @brief List of prisms in the Risley beam deflector
   * Each prism has its own properties such as angle, refractive index, and
   * rotation speed.
   */
  std::vector<Prism> prisms;

  /**
   * @brief Incident beam direction
   * This is the initial direction of the beam before it interacts with the
   * prisms.
   */
  glm::dvec3 incidentBeam;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for risley prisms beam deflector
   * @see RisleyBeamDeflector::cfg_device_scanProduct
   * @see AbstractBeamDeflector::AbstractBeamDeflector(
   *  double, double, double)
   */
  RisleyBeamDeflector(const std::vector<Prism>& prisms,
                      double refrIndex_air,
                      glm::dvec3 incidentBeam = Directions::forward)
    : AbstractBeamDeflector(0, 0, 0)
    , prisms(prisms)
    , refrIndex_air(refrIndex_air)
    , incidentBeam(incidentBeam)
  {
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
  std::string getOpticsType() const override { return "RISLEY"; }
};
