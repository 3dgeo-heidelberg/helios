#include <maths/EnergyModel.h>
#include <maths/MathConstants.h>
#include <scanner/ScanningDevice.h>
#include <util/HeliosException.h>
#include <util/logger/logging.hpp>

#include <cmath>
#include <limits>

namespace {
// Angular reflectance response, including incidence cosine where applicable.
double
computeAngularReflectance(Material const& mat, double incidenceCosine)
{
  if (mat.isPhong()) {
    double const cosDoubleAngle = 2.0 * incidenceCosine * incidenceCosine - 1.0;
    return mat.reflectance *
           ((1.0 - mat.specularity) * incidenceCosine +
            mat.specularity *
              std::pow(std::abs(cosDoubleAngle), mat.specularExponent));
  } else if (mat.isLambert()) {
    return mat.reflectance * incidenceCosine;
  } else if (mat.isDirectionIndependent()) {
    return mat.reflectance;
  }
  logging::ERR("Unexpected lighting model for material \"" + mat.name + "\"");
  throw HeliosException("Unexpected lighting model.");
}

double
computeAtmosphericTransmission(double targetRange_m, double extinction_per_m)
{
  return std::exp(-2.0 * targetRange_m * extinction_per_m);
}
#if DATA_ANALYTICS >= 2
double
computeCrossSection(double angularReflectance, double area_m2)
{
  return PI_4 * angularReflectance * area_m2;
}
#endif
}

EnergyModel::EnergyModel(ScanningDevice const& device)
  : sd(device)
{
}

double
EnergyModel::computeReceivedPower(
  double incidenceCosine,
  double targetRange,
  Material const& mat,
  std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& radiometryRecords
#endif
) const
{
  return sd.pulseAmplitude_W * computePowerRatio(incidenceCosine,
                                                 targetRange,
                                                 mat,
                                                 subrayIndex
#if DATA_ANALYTICS >= 2
                                                 ,
                                                 radiometryRecords
#endif
                               );
}

double
EnergyModel::computePowerRatio(
  double incidenceCosine,
  double targetRange,
  Material const& mat,
  std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& radiometryRecords
#endif
) const
{
  double const rangeSquared = targetRange * targetRange;
  double const share = sd.getSubrays().at(subrayIndex).share;
  double const angularReflectance =
    computeAngularReflectance(mat, incidenceCosine);
  double const atmosphere =
    computeAtmosphericTransmission(targetRange, sd.atmosphericExtinction);
  // sigma = 4*pi*angularReflectance*A cancels A in the extended-target
  // equation.
  double const powerRatio = PI * share * sd.cached_Dr2 * sd.efficiency *
                            atmosphere * angularReflectance /
                            (4.0 * rangeSquared);
#if DATA_ANALYTICS >= 2
  double const area = computeTargetArea(targetRange, subrayIndex);
  radiometryRecords.emplace_back(13, std::numeric_limits<double>::quiet_NaN());
  auto& record = radiometryRecords.back();
  auto const& subray = sd.getSubrays().at(subrayIndex);
  record[6] = targetRange * std::sin(subray.angle_rad);
  record[3] = std::acos(incidenceCosine);
  record[4] = targetRange;
  record[5] = area;
  record[7] = angularReflectance;
  record[8] = computeCrossSection(angularReflectance, area);
  record[9] = sd.pulseAmplitude_W * powerRatio;
  record[10] = 0;
  record[11] = sd.pulseAmplitude_W * share;
  record[12] = subrayIndex;
#endif
  return powerRatio;
}

double
EnergyModel::computeReceivedPowerWithSigma(double targetRange,
                                           double sigma,
                                           std::size_t subrayIndex) const
{
  return sd.pulseAmplitude_W *
         computePowerRatioWithSigma(targetRange, sigma, subrayIndex);
}

double
EnergyModel::computePowerRatioWithSigma(double targetRange,
                                        double sigma,
                                        std::size_t subrayIndex) const
{
  double const rangeSquared = targetRange * targetRange;
  double const share = sd.getSubrays().at(subrayIndex).share;
  double const area = computeTargetArea(targetRange, subrayIndex);
  double const atmosphere =
    computeAtmosphericTransmission(targetRange, sd.atmosphericExtinction);
  return share * sd.cached_Dr2 * sd.efficiency * atmosphere * sigma /
         (16.0 * area * rangeSquared);
}

double
EnergyModel::computeEmittedPower(std::size_t subrayIndex) const
{
  return sd.pulseAmplitude_W * sd.getSubrays().at(subrayIndex).share;
}

double
EnergyModel::computeTargetArea(double targetRange_m,
                               std::size_t subrayIndex) const
{
  auto const& subray = sd.getSubrays().at(subrayIndex);
  double const cosine = std::cos(subray.angle_rad);
  return subray.areaFactor * targetRange_m * targetRange_m * cosine * cosine;
}
