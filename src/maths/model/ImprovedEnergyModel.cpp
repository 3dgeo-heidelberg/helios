#include <ImprovedEnergyModel.h>
#include <maths/EnergyMaths.h>
#include <logging.hpp>
#include <scanner/ScanningDevice.h>
#include <scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ImprovedEnergyModel::ImprovedEnergyModel(ScanningDevice const& sd)
  : BaseEnergyModel(sd)
  , radii(sd.FWF_settings.beamSampleQuality + 1)
  , radiiSquared(sd.FWF_settings.beamSampleQuality + 1)
  , negRadiiSquaredx2(sd.FWF_settings.beamSampleQuality + 1)
  , w0Squared((sd.beamQuality * sd.wavelength_m) *
              (sd.beamQuality * sd.wavelength_m) /
              ((M_PI * sd.beamDivergence_rad) * (M_PI * sd.beamDivergence_rad)))
  , totPower(2 * sd.averagePower_w / (M_PI * w0Squared))
  , omegaCacheSquared((sd.wavelength_m / (M_PI * w0Squared)) *
                      (sd.wavelength_m / (M_PI * w0Squared)))
  , targetAreaCache(sd.FWF_settings.beamSampleQuality)
  , deviceConstantExpression(sd.FWF_settings.beamSampleQuality)
{
  // Cached radii
  int const BSQ = sd.FWF_settings.beamSampleQuality;
  radii[0] = 0.0;
  radiiSquared[0] = 0.0;
  negRadiiSquaredx2[0] = 0.0;
  for (int i = 0; i < BSQ; ++i) {
    int const subraysAtRing = (i == 0) ? 1 : (int)(i * PI_2);
    radii[i + 1] = sd.beamDivergence_rad * (i + 0.5) / (2 * (BSQ - 0.5));
    radiiSquared[i + 1] = radii[i + 1] * radii[i + 1];
    negRadiiSquaredx2[i + 1] = -2.0 * radiiSquared[i + 1];
    targetAreaCache[i] = M_PI / ((double)subraysAtRing);
    deviceConstantExpression[i] =
      M_PI * totPower * w0Squared / (2.0 * ((double)subraysAtRing));
  }
}

// ***  METHODS  *** //
// ***************** //
double
ImprovedEnergyModel::computeIntensity(
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
  ImprovedReceivedPowerArgs args = ImprovedReceivedPowerArgs(
    targetRange, incidenceAngle, mat, subrayRadiusStep);
  return computeReceivedPower(args
#if DATA_ANALYTICS >= 2
                              ,
                              calcIntensityRecords
#endif
  );
}

double
ImprovedEnergyModel::computeReceivedPower(
  ModelArg const& _args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  (void)_args;
#if DATA_ANALYTICS >= 2
  (void)calcIntensityRecords;
#endif
  // No geometry/atmosphere/BDRF; just pass through constant emit power.
  return computeEmittedPower(
    ImprovedEmittedPowerArgs{ 0.0, 0.0, sd.detector->cfg_device_rangeMin_m, 0 });
}

double
ImprovedEnergyModel::computeEmittedPower(ModelArg const& _args)
{
  ImprovedEmittedPowerArgs const& args =
    static_cast<ImprovedEmittedPowerArgs const&>(_args);
  (void)args; // unused for constant energy model
  // Give every subray the full average power (no Gaussian weighting, no split).
  return sd.averagePower_w;
}

double
ImprovedEnergyModel::computeTargetArea(
  ModelArg const& _args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  // Once for target area and once for emitted power
  ImprovedTargetAreaArgs const& args =
    static_cast<ImprovedTargetAreaArgs const&>(_args);
  double const prevRadiusSquared = radiiSquared[args.subrayRadiusStep];
  double const radiusSquared = radiiSquared[args.subrayRadiusStep + 1];
  double const radius_m_squared = radiusSquared * args.targetRangeSquared;
  double const prevRadius_m_squared =
    prevRadiusSquared * args.targetRangeSquared;
#if DATA_ANALYTICS >= 2
  std::vector<double> calcIntensityRecord(
    13, std::numeric_limits<double>::quiet_NaN());
  calcIntensityRecord[6] = std::sqrt(radius_m_squared);
  calcIntensityRecords.push_back(calcIntensityRecord);
#endif
  return (radius_m_squared - prevRadius_m_squared) *
         targetAreaCache[args.subrayRadiusStep];
}
