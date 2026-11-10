#pragma once

#include <cstddef>
#include <scene/Material.h>
#include <vector>

class ScanningDevice;

/** Far-field radiometry using the device's absolute subray power shares.
 * Ranges are metres; incidenceCosine is in [-1, 1]. Power is watts.
 * Sigma retains the external scattering convention, including LadLut inputs.
 */
class EnergyModel
{
  ScanningDevice const& sd;

public:
  explicit EnergyModel(ScanningDevice const& device);
  /** Received/whole-beam emitted peak-power ratio, including subray share. */
  double computePowerRatio(double incidenceCosine,
                           double targetRange_m,
                           Material const& material,
                           std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
                           ,
                           std::vector<std::vector<double>>& radiometryRecords
#endif
  ) const;
  /** Received peak power in watts. */
  double computeReceivedPower(
    double incidenceCosine,
    double targetRange_m,
    Material const& material,
    std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& radiometryRecords
#endif
  ) const;
  /** Dimensionless power ratio for externally supplied cross section sigma. */
  double computePowerRatioWithSigma(double targetRange_m,
                                    double sigma,
                                    std::size_t subrayIndex) const;
  /** Received peak power in watts for externally supplied cross section sigma.
   */
  double computeReceivedPowerWithSigma(double targetRange_m,
                                       double sigma,
                                       std::size_t subrayIndex) const;
  /** Whole-beam emitted peak power times absolute subray share, in watts. */
  double computeEmittedPower(std::size_t subrayIndex) const;
  /** Transverse subray patch area in square metres.
   */
  double computeTargetArea(double targetRange_m, std::size_t subrayIndex) const;
};
