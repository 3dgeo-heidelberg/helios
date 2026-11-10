#include "Material.h"
#include <stdexcept>

void
Material::validate() const
{
  if (!std::isfinite(specularity) || specularity < 0 || specularity > 1)
    throw std::invalid_argument("Material " + name +
                                ": specularity must be finite and in [0, 1]");
  if (!std::isfinite(specularExponent) || specularExponent < 0)
    throw std::invalid_argument(
      "Material " + name +
      ": specular exponent must be finite and nonnegative");
}
