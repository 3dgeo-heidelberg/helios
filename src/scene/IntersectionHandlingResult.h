#pragma once

#include <glm/glm.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Output class for intersection handling methods
 */
class IntersectionHandlingResult
{
private:
  // *********************** //

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief True if the ray can continue after intersection, False otherwise
   */
  glm::dvec3 intersectionPoint;
  bool canContinue;
  /**
   * @brief Incidence angle (radians) determined by the intersection
   * handling, e.g., sampled from a leaf angle distribution. NaN if the
   * handling does not determine it.
   */
  double incidenceAngle_rad;

public:
  // ***  CONSTRUCTION  *** //
  // ********************** //
  /**
   * @brief Build an IntersectionHandlingResult
   * @param intersectionPoint Specify the intersection point
   * @param canContinue Specify if the ray can continue after intersection
   * or not
   */
  IntersectionHandlingResult(
    glm::dvec3 intersectionPoint = glm::dvec3(0, 0, 0),
    bool canContinue = false,
    double incidenceAngle_rad = std::numeric_limits<double>::quiet_NaN())
    : intersectionPoint(std::move(intersectionPoint))
    , canContinue(canContinue)
    , incidenceAngle_rad(incidenceAngle_rad)
  {
  }

  // ***  M E T H O D S  *** //
  // ************************ //
  /**
   * @brief Check whether the ray can continue after intersection or not
   * @return True if the ray can continue after intersection, False if not
   */
  inline bool canRayContinue() { return canContinue; }
  /**
   * @brief Obtain the intersection point
   * @return Intersection point
   */
  glm::dvec3 getIntersectionPoint() { return intersectionPoint; }
  /**
   * @brief Check whether the intersection handling determined the
   * incidence angle
   */
  inline bool hasIncidenceAngle() const
  {
    return !std::isnan(incidenceAngle_rad);
  }
  /**
   * @brief Obtain the incidence angle determined by the intersection
   * handling (radians), NaN if none
   * @see IntersectionHandlingResult::hasIncidenceAngle
   */
  inline double getIncidenceAngle_rad() const { return incidenceAngle_rad; }

  // ***  STREAM OPERATORS  *** //
  // ************************** //
  friend std::ostream& operator<<(std::ostream& out,
                                  IntersectionHandlingResult const& ihr);
};
