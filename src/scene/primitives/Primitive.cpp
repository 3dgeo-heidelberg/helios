#include "Primitive.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
void
Primitive::_clone(Primitive* p)
{
  if (this->material == nullptr)
    p->material = nullptr;
  else
    p->material = std::make_shared<Material>(*this->material);
}

// ***  RAY INTERSECTION HANDLING  *** //
// *********************************** //
IntersectionHandlingResult
Primitive::onRayIntersection(NoiseSource<double>& uniformNoiseSource,
                             glm::dvec3& rayDirection,
                             glm::dvec3 const& insideIntersectionPoint,
                             glm::dvec3 const& outsideIntersectionPoint,
                             double rayIntensity)
{
  return IntersectionHandlingResult();
}

// ***  STREAM OPERATORS  *** //
// ************************** //
std::ostream&
operator<<(std::ostream& out, Primitive& p)
{
  std::stringstream ss;
  ss << std::setw(16) << std::fixed;

  ss << "Primitive:";

  for (size_t i = 0; i < p.getNumVertices(); i++) {
    ss << "\nv" << i << ": (" << p.getVertices()[i].getX() << ", "
       << p.getVertices()[i].getY() << ", " << p.getVertices()[i].getZ() << ")";
  }

  ss << "\n";

  out << ss.str();
  return out;
}

// ***  TRANSFORMATIONS  *** //
// ************************* //
void
Primitive::rotate(Rotation& r)
{
  for (size_t i = 0; i < getNumVertices(); i++) {
    getVertices()[i].pos = r.applyTo(getVertices()[i].pos);
    getVertices()[i].normal = r.applyTo(getVertices()[i].normal);
  }
}
void
Primitive::scale(double const factor)
{
  for (size_t i = 0; i < getNumVertices(); i++) {
    getVertices()[i].pos = glm::dvec3(getVertices()[i].pos.x * factor,
                                      getVertices()[i].pos.y * factor,
                                      getVertices()[i].pos.z * factor);
  }
}
void
Primitive::translate(glm::dvec3 const& shift)
{
  for (size_t i = 0; i < getNumVertices(); i++) {
    getVertices()[i].pos = getVertices()[i].pos + shift;
  }
}
