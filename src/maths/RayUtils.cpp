#include <HeliosException.h>
#include <logging.hpp>
#include <maths/RayUtils.h>

glm::dvec3
RayUtils::obtainPointAfterTraversing(AABB const& aabb,
                                     glm::dvec3 const& origin,
                                     glm::dvec3 const& direction,
                                     double eps)
{
  // Exctract bounding box coordinates
  glm::dvec3 const& aabbMin = aabb.getMin();
  glm::dvec3 const& aabbMax = aabb.getMax();

  // Find intersection times
  double tx = (aabbMin.x - origin.x) / direction.x;
  double aux = (aabbMax.x - origin.x) / direction.x;
  if (aux > tx)
    tx = aux;
  double ty = (aabbMin.y - origin.y) / direction.y;
  aux = (aabbMax.y - origin.y) / direction.y;
  if (aux > ty)
    ty = aux;
  double tz = (aabbMin.z - origin.z) / direction.z;
  aux = (aabbMax.z - origin.z) / direction.z;
  if (aux > tz)
    tz = aux;

  // Find last intersection time
  double tmax = tx;
  if (ty != std::numeric_limits<double>::infinity() &&
      (ty < tmax || tmax == std::numeric_limits<double>::infinity()))
    tmax = ty;
  if (tz != std::numeric_limits<double>::infinity() &&
      (tz < tmax || tmax == std::numeric_limits<double>::infinity()))
    tmax = tz;
  tmax += eps; // Little offset to get outside
  if (tmax <= 0.0) {
    throw HeliosException(
      "RayUtils::obtainPointerAfterTraversing function: tmax <= 0");
  }

  // Compute and return the after traversing point
  return origin + tmax * direction;
}
