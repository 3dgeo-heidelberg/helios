#include "Material.h"
#include <stdexcept>

void
Material::validate() const
{
  if (!std::isnan(reflectance) &&
      (!std::isfinite(reflectance) || reflectance < 0 || reflectance > 1))
    throw std::invalid_argument("Material reflectance must be in [0, 1]");
  if (!std::isfinite(ks) || ks < 0 || ks > 1)
    throw std::invalid_argument("Material " + name +
                                ": ks must be finite and in [0, 1]");
  if (!std::isfinite(specularExponent) || specularExponent < 0)
    throw std::invalid_argument(
      "Material " + name +
      ": specular exponent must be finite and nonnegative");
}
