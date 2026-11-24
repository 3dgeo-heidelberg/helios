#pragma once

#include <algorithm>
#include <cmath>
#include <maths/MathConstants.h>
#include <scene/Material.h>

// Lafortune and Willems (1994). BRDF in sr^-1, without projected-area cosine.
// Material is validated by the caller. q is the mirror/outgoing dot product.
inline double
modifiedPhongBrdf(Material const& material, double q)
{
  double const lobe =
    material.specularExponent == 0
      ? 1.0
      : std::pow(std::clamp(q, 0.0, 1.0), material.specularExponent);
  return material.reflectance *
         ((1.0 - material.ks) / PI +
          material.ks * (material.specularExponent + 2.0) / (2.0 * PI) * lobe);
}
