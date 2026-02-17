#include <helios/maths/EnergyMaths.h>
#include <helios/maths/model/BaseEnergyModel.h>
#include <helios/scanner/ScanningDevice.h>
#include <helios/scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
BaseEnergyModel::BaseEnergyModel(ScanningDevice const& sd)
  : EnergyModel(sd)
{
  // Precompute cached values
  targetAreaCache = PI_QUARTER * sd.cached_Bt2 / sd.numRays;
}

// ***  METHODS  *** //
// ***************** //
double
BaseEnergyModel::computeIntensity(
  double const incidenceAngle,
  double const targetRange,
  Material const& mat,
  int const subrayRadiusStep
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  BaseReceivedPowerArgs args = BaseReceivedPowerArgs(
    incidenceAngle,
    targetRange,
    mat,
    targetRange *
      std::sin(sd.cached_subrayDivergenceAngle_rad[subrayRadiusStep]));
  return computeReceivedPower(args
#if DATA_ANALYTICS >= 2
                              ,
                              calcIntensityRecords
#endif
  );
}

double
BaseEnergyModel::computeReceivedPower(
  ModelArg const& _args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  BaseReceivedPowerArgs const& args =
    static_cast<BaseReceivedPowerArgs const&>(_args);
  // Pre-computations
  double const squaredRange = args.targetRange * args.targetRange;
  // Target area
  double const targetArea = computeTargetArea(BaseTargetAreaArgs{ squaredRange }
#if DATA_ANALYTICS >= 2
                                              ,
                                              calcIntensityRecords
#endif
  );
  // BDRF
  double const bdrf =
    EnergyMaths::computeBDRF(args.material, args.incidenceAngle_rad);
  // Cross-section
  double const sigma = computeCrossSection(
    BaseCrossSectionArgs{ args.material, bdrf, targetArea });
  // Received power
  double const receivedPower = EnergyMaths::calcReceivedPowerFast(
    sd.averagePower_w,
    sd.wavelength_m * sd.wavelength_m,
    args.targetRange,
    squaredRange,
    sd.detector->cfg_device_rangeMin_m * sd.detector->cfg_device_rangeMin_m,
    args.subrayRadius * args.subrayRadius,
    sd.beamWaistRadius * sd.beamWaistRadius,
    sd.cached_Dr2,
    sd.cached_Bt2,
    sd.efficiency,
    sd.atmosphericExtinction,
    sigma);
#if DATA_ANALYTICS >= 2
  std::vector<double> calcIntensityRecord(
    13, std::numeric_limits<double>::quiet_NaN());
  calcIntensityRecord[3] = args.incidenceAngle_rad;
  calcIntensityRecord[4] = args.targetRange;
  calcIntensityRecord[5] = targetArea;
  calcIntensityRecord[6] = args.subrayRadius;
  calcIntensityRecord[7] = bdrf;
  calcIntensityRecord[8] = sigma;
  calcIntensityRecord[9] = receivedPower;
  calcIntensityRecord[10] = 0; // By default, assume the point isn't captured
  calcIntensityRecord[11] =
    EnergyMaths::calcEmittedPower(sd.averagePower_w,
                                  sd.wavelength_m,
                                  args.targetRange,
                                  sd.detector->cfg_device_rangeMin_m,
                                  args.subrayRadius,
                                  sd.beamWaistRadius);
  calcIntensityRecord[12] = -1.0;
  calcIntensityRecords.push_back(calcIntensityRecord);
#endif
  return receivedPower * 1e09;
}

double
BaseEnergyModel::computeEmittedPower(ModelArg const& _args)
{
  BaseEmittedPowerArgs const& args =
    static_cast<BaseEmittedPowerArgs const&>(_args);
  return EnergyMaths::calcEmittedPower(args.averagePower_w,
                                       args.wavelength_m,
                                       args.targetRange,
                                       args.rangeMin,
                                       args.subrayRadius,
                                       args.beamWaistRadius);
}

double
BaseEnergyModel::computeTargetArea(
  ModelArg const& _args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  BaseTargetAreaArgs const& args =
    static_cast<BaseTargetAreaArgs const&>(_args);
  return targetAreaCache * args.squaredRange;
}

double
BaseEnergyModel::computeCrossSection(ModelArg const& _args)
{
  BaseCrossSectionArgs const& args =
    static_cast<BaseCrossSectionArgs const&>(_args);
  return EnergyMaths::calcCrossSection(args.bdrf, args.targetArea);
}
