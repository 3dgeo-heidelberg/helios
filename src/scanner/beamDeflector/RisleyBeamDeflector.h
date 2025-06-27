#pragma once

#include <string>

#include "AbstractBeamDeflector.h"

#include "maths/MathConverter.h"

#include <glm/glm.hpp>

/**
 * @brief Class representing a Risley-prism beam deflector based on three
 * independently rotating prisms according to Anhu Li, Xingsheng Liu, and
 * Wansong Sun, "Forward and inverse solutions for three-element Risley prism
 * beam scanners," Opt. Express 25, 7677-7688 (2017), DOI: 10.1364/OE.25.007677
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
   * @brief Scan Angle (defined as the half angle)
   */
  double scanAngle = 0;
  double rotorSpeed_rad_1 = 0;
  double rotorSpeed_rad_2 = 0;
  double rotorSpeed_rad_3 = 0;
  double prism1_angle_rad = 0;
  double prism2_angle_rad = 0;
  double prism3_angle_rad = 0;
  double prism1_thickness_base = 0;
  double prism2_thickness_base = 0;
  double prism3_thickness_base = 0;
  double prism1_radius = 0;
  double prism2_radius = 0;
  double prism3_radius = 0;
  double distance_prism1_2 = 0;
  double distance_prism2_3 = 0;
  double refrIndex_prism1 = 0;
  double refrIndex_prism2 = 0;
  double refrIndex_prism3 = 0;
  double refrIndex_air = 0;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for risley prisms beam deflector
   * @see RisleyBeamDeflector::cfg_device_scanProduct
   * @see AbstractBeamDeflector::AbstractBeamDeflector(
   *  double, double, double)
   */
  RisleyBeamDeflector(double scanAngleMax_rad,
                      double rotorFreq_Hz_1,
                      double rotorFreq_Hz_2,
                      double rotorFreq_Hz_3,
                      double prism1_angle_deg,
                      double prism2_angle_deg,
                      double prism3_angle_deg,
                      double prism1_thickness_base,
                      double prism2_thickness_base,
                      double prism3_thickness_base,
                      double prism1_radius,
                      double prism2_radius,
                      double prism3_radius,
                      double distance_prism1_2,
                      double distance_prism2_3,
                      double refrIndex_prism1,
                      double refrIndex_prism2,
                      double refrIndex_prism3,
                      double refrIndex_air)
    : AbstractBeamDeflector(scanAngleMax_rad, 0, 0)
  {
    this->scanAngle = scanAngleMax_rad;
    this->rotorSpeed_rad_1 = rotorFreq_Hz_1 * 0.5 / M_PI;
    this->rotorSpeed_rad_2 = rotorFreq_Hz_2 * 0.5 / M_PI;
    this->rotorSpeed_rad_3 = rotorFreq_Hz_3 * 0.5 / M_PI;

    this->prism1_angle_rad = MathConverter::degreesToRadians(prism1_angle_deg);
    this->prism2_angle_rad = MathConverter::degreesToRadians(prism2_angle_deg);
    this->prism3_angle_rad = MathConverter::degreesToRadians(prism3_angle_deg);

    this->prism1_thickness_base = prism1_thickness_base;
    this->prism2_thickness_base = prism2_thickness_base;
    this->prism3_thickness_base = prism3_thickness_base;

    this->prism1_radius = prism1_radius;
    this->prism2_radius = prism2_radius;
    this->prism3_radius = prism3_radius;

    this->distance_prism1_2 = distance_prism1_2;
    this->distance_prism2_3 = distance_prism2_3;

    this->refrIndex_prism1 = refrIndex_prism1;
    this->refrIndex_prism2 = refrIndex_prism2;
    this->refrIndex_prism3 = refrIndex_prism3;
    this->refrIndex_air = refrIndex_air;

    initializeGeometry();
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

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @see AbstractBeamDeflector::setScanAngle_rad
   */
  void setScanAngle_rad(double scanAngle_rad) override;
  /**
   * @see AbstractBeamDeflector::setScanFreq_Hz
   */
  void setScanFreq_Hz(double scanFreq_Hz) override;

private:
  // Incident beam
  glm::dvec3 originalBeam = glm::dvec3(0.0, 0.0, 1.0);

  // Point on incident beam
  glm::dvec3 pointOnOriginalBeam;

  // Points on all surfaces on z-axis
  glm::dvec3 Prism1Point1ZAxis, Prism1Point2ZAxis;
  glm::dvec3 Prism2Point1ZAxis, Prism2Point2ZAxis;
  glm::dvec3 Prism3Point1ZAxis, Prism3Point2ZAxis;

  // Static prism normal vectors (vertical side)
  glm::dvec3 Prism1NormalVector2 = glm::dvec3(0.0, 0.0, -1.0);
  glm::dvec3 Prism2NormalVector1 = glm::dvec3(0.0, 0.0, -1.0);
  glm::dvec3 Prism3NormalVector2 = glm::dvec3(0.0, 0.0, -1.0);

  void initializeGeometry();

  /**
   * @brief Computes the intersection point between a line and a plane in 3D.
   *
   * Given a line defined by a point and direction vector, and a plane defined
   * by a point and normal vector, this function computes the intersection point
   * (if it exists) where the line crosses the plane.
   *
   * @param pointOnLine A point on the line (glm::dvec3).
   * @param lineDirection The direction vector of the line (glm::dvec3).
   * @param pointOnPlane A point on the plane (glm::dvec3).
   * @param normalPlane The normal vector of the plane (glm::dvec3).
   * @param intersection Output parameter: the intersection point (glm::dvec3).
   *
   * @return true if the line intersects the plane (not parallel), false
   * otherwise.
   *
   * @note The function does not check if the intersection point lies within a
   * bounded surface. It assumes the line is infinite in both directions and the
   * plane is infinite.
   */
  bool intersectLinePlane(const glm::dvec3& pointOnLine,
                          const glm::dvec3& lineDirection,
                          const glm::dvec3& pointOnPlane,
                          const glm::dvec3& normalPlane,
                          glm::dvec3& intersection);

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
  static bool refractBeam(const glm::dvec3& incidentBeamDirection,
                          const glm::dvec3& surfaceNormal,
                          double refractiveIdxA,
                          double refractiveIdxB,
                          glm::dvec3& refracted);
};
