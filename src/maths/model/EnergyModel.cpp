#include <EnergyModel.h>
#include <maths/EnergyMaths.h>
#include <scanner/ScanningDevice.h>

#include <cmath>
#include <limits>

EnergyModel::EnergyModel(ScanningDevice const& sd)
  : sd(sd)
{
}

double
EnergyModel::computeIntensity(
  double incidenceAngle,
  double targetRange,
  Material const& mat,
  std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  return computeReceivedPower(
    ReceivedPowerArgs{ targetRange, incidenceAngle, mat, subrayIndex }
#if DATA_ANALYTICS >= 2
    ,
    calcIntensityRecords
#endif
  );
}

double
EnergyModel::computeReceivedPower(
  ReceivedPowerArgs const& args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  double const intensity =
    computeIntensityFromCosine(std::cos(args.incidenceAngle_rad),
                               args.targetRange,
                               args.material,
                               args.subrayIndex
#if DATA_ANALYTICS >= 2
                               ,
                               calcIntensityRecords
#endif
    );
#if DATA_ANALYTICS >= 2
  // Preserve the supplied angle for callers of the legacy angle API.
  calcIntensityRecords.back()[3] = args.incidenceAngle_rad;
#endif
  return intensity;
}

double
EnergyModel::computeIntensityFromCosine(
  double incidenceCosine,
  double targetRange,
  Material const& mat,
  std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  double const rangeSquared = targetRange * targetRange;
  double const emittedPower =
    computeEmittedPower(EmittedPowerArgs{ subrayIndex });
  double const brdf = EnergyMaths::computeBRDFFromCosine(mat, incidenceCosine);
  double const atmosphere =
    EnergyMaths::calcAtmosphericFactor(targetRange, sd.atmosphericExtinction);
  // sigma = 4*pi*BRDF*A cancels A in the extended-target equation.
  double const receivedPower = PI * emittedPower * sd.cached_Dr2 *
                               sd.efficiency * atmosphere * brdf /
                               (4.0 * rangeSquared);
#if DATA_ANALYTICS >= 2
  double const area = computeTargetArea(
    TargetAreaArgs{ rangeSquared, subrayIndex }, calcIntensityRecords);
  auto& record = calcIntensityRecords.back();
  record[3] = std::acos(incidenceCosine);
  record[4] = targetRange;
  record[5] = area;
  record[7] = brdf;
  record[8] = EnergyMaths::calcCrossSection(brdf, area);
  record[9] = receivedPower;
  record[10] = 0;
  record[11] = emittedPower;
  record[12] = subrayIndex;
#endif
  return receivedPower * 1e09;
}

double
EnergyModel::computeReceivedPowerWithSigma(double targetRange,
                                           double sigma,
                                           std::size_t subrayIndex)
{
  double const rangeSquared = targetRange * targetRange;
  double const emittedPower =
    computeEmittedPower(EmittedPowerArgs{ subrayIndex });
#if DATA_ANALYTICS >= 2
  std::vector<std::vector<double>> unusedRecords;
  double const area = computeTargetArea(
    TargetAreaArgs{ rangeSquared, subrayIndex }, unusedRecords);
#else
  double const area =
    computeTargetArea(TargetAreaArgs{ rangeSquared, subrayIndex });
#endif
  double const atmosphere =
    EnergyMaths::calcAtmosphericFactor(targetRange, sd.atmosphericExtinction);
  return EnergyMaths::calcReceivedPowerFast(emittedPower,
                                            sd.cached_Dr2,
                                            16.0 * area * rangeSquared,
                                            sd.efficiency,
                                            atmosphere,
                                            sigma) *
         1e09;
}

double
EnergyModel::computeEmittedPower(EmittedPowerArgs const& args)
{
  return sd.averagePower_w * sd.getSubrays().at(args.subrayIndex).share;
}

double
EnergyModel::computeTargetArea(
  TargetAreaArgs const& args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  auto const& subray = sd.getSubrays().at(args.subrayIndex);
  double const cosine = std::cos(subray.angle_rad);
  double const axialRangeSquared = args.targetRangeSquared * cosine * cosine;
#if DATA_ANALYTICS >= 2
  std::vector<double> record(13, std::numeric_limits<double>::quiet_NaN());
  record[6] = std::sqrt(axialRangeSquared) * std::tan(subray.outerAngle_rad);
  calcIntensityRecords.push_back(std::move(record));
#endif
  return subray.areaFactor * axialRangeSquared;
}

double
EnergyModel::computeCrossSection(CrossSectionArgs const& args)
{
  return EnergyMaths::calcCrossSection(args.brdf, args.targetArea);
}
