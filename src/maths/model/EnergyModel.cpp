#include <EnergyModel.h>
#include <maths/EnergyMaths.h>
#include <scanner/ScanningDevice.h>
#include <scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
EnergyModel::EnergyModel(ScanningDevice const& sd)
  : sd(sd)
  , radii(sd.FWF_settings.beamSampleQuality + 1)
  , radiiSquared(sd.FWF_settings.beamSampleQuality + 1)
  , negRadiiSquaredx2(sd.FWF_settings.beamSampleQuality + 1)
  , w0Squared((sd.beamQuality * sd.wavelength_m) *
              (sd.beamQuality * sd.wavelength_m) /
              ((PI * sd.beamDivergence_rad) * (PI * sd.beamDivergence_rad)))
  , totPower(2 * sd.averagePower_w / (PI * w0Squared))
  , omegaCacheSquared((sd.wavelength_m / (PI * w0Squared)) *
                      (sd.wavelength_m / (PI * w0Squared)))
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
    targetAreaCache[i] = PI / ((double)subraysAtRing);
    deviceConstantExpression[i] =
      PI * totPower * w0Squared / (2.0 * ((double)subraysAtRing));
  }
}

// ***  METHODS  *** //
// ***************** //
double
EnergyModel::computeIntensity(
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
  ReceivedPowerArgs args =
    ReceivedPowerArgs(targetRange, incidenceAngle, mat, subrayRadiusStep);
  return computeReceivedPower(args
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
  // Pre-computations
  double const rangeSquared = args.targetRange * args.targetRange;
  // Emitted power
  double const Pe =
    computeEmittedPower(EmittedPowerArgs{ args.targetRange,
                                          rangeSquared,
                                          sd.detector->cfg_device_rangeMin_m,
                                          args.subrayRadiusStep });
  // Target area
  double const targetArea =
    computeTargetArea(TargetAreaArgs{ rangeSquared, args.subrayRadiusStep }
#if DATA_ANALYTICS >= 2
                      ,
                      calcIntensityRecords
#endif
    );
  // Cross-section
  double const bdrf =
    EnergyMaths::computeBDRF(args.material, args.incidenceAngle_rad);
  double const sigma =
    computeCrossSection(CrossSectionArgs{ args.material, bdrf, targetArea });
  // Received power
  double const atmosphericFactor = EnergyMaths::calcAtmosphericFactor(
    args.targetRange, sd.atmosphericExtinction);
  double const receivedPower =
    EnergyMaths::calcReceivedPowerImprovedFast(Pe,
                                               sd.cached_Dr2,
                                               16 * targetArea * rangeSquared,
                                               sd.efficiency,
                                               atmosphericFactor,
                                               sigma);
#if DATA_ANALYTICS >= 2
  std::vector<double>& calcIntensityRecord = calcIntensityRecords.back();
  calcIntensityRecord[3] = args.incidenceAngle_rad;
  calcIntensityRecord[4] = args.targetRange;
  calcIntensityRecord[5] = targetArea;
  calcIntensityRecord[7] = bdrf;
  calcIntensityRecord[8] = sigma;
  calcIntensityRecord[9] = receivedPower;
  calcIntensityRecord[10] = 0; // By default, assume the point isn't captured
  calcIntensityRecord[11] = Pe;
  calcIntensityRecord[12] = args.subrayRadiusStep;
#endif
  return receivedPower * 1e09;
}

double
EnergyModel::computeEmittedPower(EmittedPowerArgs const& args)
{
  double const Omega0 = 1 - args.targetRange / args.rangeMin;
  double const OmegaSquared = args.targetRangeSquared * omegaCacheSquared;
  double const wSquared = w0Squared * (Omega0 * Omega0 + OmegaSquared);
  return EnergyMaths::calcSubrayWiseEmittedPowerFast(
    deviceConstantExpression[args.subrayRadiusStep],
    wSquared,
    negRadiiSquaredx2[args.subrayRadiusStep + 1],
    negRadiiSquaredx2[args.subrayRadiusStep]);
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
  // Once for target area and once for emitted power
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

double
EnergyModel::computeCrossSection(CrossSectionArgs const& args)
{
  return EnergyMaths::calcCrossSection(args.bdrf, args.targetArea);
}
