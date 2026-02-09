#pragma once

// This works around a known issue in boost:
// https://github.com/boostorg/serialization/issues/315
#ifdef BOOST_NO_EXCEPTIONS
#include <boost/throw_exception.hpp>
#endif
#include <boost/serialization/serialization.hpp>
#include <glm/glm.hpp>
#include <iostream>
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
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize an IntersectionHandlingResult to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the intersection handling result
   */
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & intersectionPoint;
    ar & canContinue;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief True if the ray can continue after intersection, False otherwise
   */
  glm::dvec3 intersectionPoint;
  bool canContinue;

public:
  // ***  CONSTRUCTION  *** //
  // ********************** //
  /**
   * @brief Build an IntersectionHandlingResult
   * @param intersectionPoint Specify the intersection point
   * @param canContinue Specify if the ray can continue after intersection
   * or not
   */
  IntersectionHandlingResult(glm::dvec3 intersectionPoint = glm::dvec3(0, 0, 0),
                             bool canContinue = false)
    : intersectionPoint(std::move(intersectionPoint))
    , canContinue(canContinue)
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

  // ***  STREAM OPERATORS  *** //
  // ************************** //
  friend std::ostream& operator<<(std::ostream& out,
                                  IntersectionHandlingResult const& ihr);
};
