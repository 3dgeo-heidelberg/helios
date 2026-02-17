#include <helios/util/PrintUtils.h>

// ***  OPERATOR <<  *** //
// ********************* //
std::ostream&
operator<<(std::ostream& out, glm::dvec3 const& v)
{
  out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return out;
}
