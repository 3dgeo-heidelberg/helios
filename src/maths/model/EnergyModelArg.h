#pragma once

class Material;

struct ReceivedPowerArgs
{
  double const targetRange;
  double const incidenceAngle_rad;
  Material const& material;
  int const subrayRadiusStep;
  ReceivedPowerArgs(double const targetRange,
                    double const incidenceAngle_rad,
                    Material const& material,
                    int const subrayRadiusStep)
    : targetRange(targetRange)
    , incidenceAngle_rad(incidenceAngle_rad)
    , material(material)
    , subrayRadiusStep(subrayRadiusStep)
  {
  }
};

struct EmittedPowerArgs
{
  double const targetRange;
  double const targetRangeSquared;
  double const rangeMin;
  int const subrayRadiusStep;
  EmittedPowerArgs(double const targetRange,
                   double const targetRangeSquared,
                   double const rangeMin,
                   int const subrayRadiusStep)
    : targetRange(targetRange)
    , targetRangeSquared(targetRangeSquared)
    , rangeMin(rangeMin)
    , subrayRadiusStep(subrayRadiusStep)
  {
  }
};

struct TargetAreaArgs
{
  double const targetRangeSquared;
  int const subrayRadiusStep;
  TargetAreaArgs(double const targetRangeSquared, int const subrayRadiusStep)
    : targetRangeSquared(targetRangeSquared)
    , subrayRadiusStep(subrayRadiusStep)
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
