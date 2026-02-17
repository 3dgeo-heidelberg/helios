#pragma once

#include <glm/glm.hpp>
#include <helios/scene/primitives/AABB.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class with util functions to work with rays.
 */
class RayUtils
{
public:
  /**
   * @brief Obtain the point immediately after finishing traversing given
   * bounding box, with an offset specified as eps (epsilon)
   *
   * \f[
   *  p = o + (t + \epsilon) \cdot \hat{v}
   * \f]
   *
   * NOTICE this function assumes intersection occurs.
   * Using this function when this assumption is not satisfied might lead
   * to unexpected behaviors and wrong output
   *
   * @param aabb The axis aligned bounding box being traversed
   * @param origin The originWaypoint of the ray
   * @param direction The direction of the ray
   * @param eps The offset specification.
   * <b>NOTICE</b> if \f$\epsilon=0\f$ then the returned point will be "exactly"
   * the intersection point corresponding to the ray leaving the bounding box
   * @return Point immediately after finishing traversing given bounding box,
   * with eps offset
   */
  static glm::dvec3 obtainPointAfterTraversing(AABB const& aabb,
                                               glm::dvec3 const& origin,
                                               glm::dvec3 const& direction,
                                               double eps = 0.00001);
};
