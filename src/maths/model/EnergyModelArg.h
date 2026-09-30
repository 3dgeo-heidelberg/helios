#pragma once

#include <cstddef>

class Material;

struct ReceivedPowerArgs
{
  double const targetRange;
  double const incidenceAngle_rad;
  Material const& material;
  std::size_t const subrayIndex;
  ReceivedPowerArgs(double const targetRange,
                    double const incidenceAngle_rad,
                    Material const& material,
                    std::size_t subrayIndex)
    : targetRange(targetRange)
    , incidenceAngle_rad(incidenceAngle_rad)
    , material(material)
    , subrayIndex(subrayIndex)
  {
  }
};

struct EmittedPowerArgs
{
  std::size_t const subrayIndex;
  explicit EmittedPowerArgs(std::size_t subrayIndex)
    : subrayIndex(subrayIndex)
  {
  }
};

struct TargetAreaArgs
{
  double const targetRangeSquared;
  std::size_t const subrayIndex;
  TargetAreaArgs(double const targetRangeSquared, std::size_t subrayIndex)
    : targetRangeSquared(targetRangeSquared)
    , subrayIndex(subrayIndex)
  {
  }
};

struct CrossSectionArgs
{
  Material const& material;
  double const bdrf; // Bidirectional reflectance function
  double const targetArea;
  CrossSectionArgs(Material const& material,
                   double const bdrf,
                   double const targetArea)
    : material(material)
    , bdrf(bdrf)
    , targetArea(targetArea)
  {
  }
};
