#pragma once

#include <maths/model/EnergyModelArg.h>
#include <scene/Material.h>
#include <vector>

class ScanningDevice;

/** Far-field radiometry using the device's shared subray power fractions. */
class EnergyModel
{
private:
  ScanningDevice const& sd;

public:
  explicit EnergyModel(ScanningDevice const& sd);

  double computeIntensity(double incidenceAngle,
                          double targetRange,
                          Material const& mat,
                          std::size_t subrayIndex
#if DATA_ANALYTICS >= 2
                          ,
                          std::vector<std::vector<double>>& calcIntensityRecords
#endif
  );
  /** Extended-target return. Public intensity units remain unchanged. */
  double computeReceivedPower(
    ReceivedPowerArgs const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
  );
  /** External sigma retains its existing patch-area dependence. */
  double computeReceivedPowerWithSigma(double targetRange,
                                       double sigma,
                                       std::size_t subrayIndex);
  /** Total emitted power times this subray's absolute Gaussian share. */
  double computeEmittedPower(EmittedPowerArgs const& args);
  /** Table-derived transverse patch area, retained for external
   * sigma/analytics. */
  double computeTargetArea(
    TargetAreaArgs const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
  );
  double computeCrossSection(CrossSectionArgs const& args);
};
