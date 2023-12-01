#include <ImprovedEnergyModel.h>
#include <maths/EnergyMaths.h>

// ***  METHODS  *** //
// ***************** //
double ImprovedEnergyModel::computeReceivedPower(
    ModelArg const & _args
#if DATA_ANALYTICS >=2
   ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
){
    ImprovedReceivedPowerArgs const &args = static_cast<
        ImprovedReceivedPowerArgs const &
    >(_args);
    double const Pe = computeEmittedPower(ImprovedEmittedPowerArgs{
        args.averagePower_w,
        args.targetRange,
        args.numSubrays,
        args.beamSampleQuality,
        args.subrayRadiusStep
    });
    double const atmosphericFactor = std::exp(
        -2 * args.targetRange * args.atmosphericExtinction
    );
    double const targetArea = computeTargetArea(ImprovedTargetAreaArgs{
            args.targetRange,
            args.deviceBeamDivergence_rad,
            args.beamSampleQuality,
            args.subrayRadiusStep
        }
#if DATA_ANALYTICS >= 2
       ,calcIntensityRecords
#endif
    );
    // TODO Rethink : To common impl, consider also BaseEnergyModel ---
    double const bdrf = EnergyMaths::computeBDRF(
        args.material,
        args.incidenceAngle_rad
    );
    double const sigma = computeCrossSection(BaseCrossSectionArgs{
        args.material,
        bdrf,
        targetArea,
        args.incidenceAngle_rad
    });
    // --- TODO Rethink : To common impl, consider also BaseEnergyModel
    double const receivedPower = EnergyMaths::calcReceivedPowerLegacy(
        Pe,
        args.Dr2,
        args.targetRange,
        args.Bt2,
        args.efficiency,
        atmosphericFactor,
        sigma
    ) * 1e9;
#if DATA_ANALYTICS >= 2
    std::vector<double> & calcIntensityRecord = calcIntensityRecords.back();
    calcIntensityRecord[3] = args.incidenceAngle_rad;
    calcIntensityRecord[4] = args.targetRange;
    calcIntensityRecord[5] = targetArea;
    calcIntensityRecord[7] = bdrf;
    calcIntensityRecord[8] = sigma;
    // TODO Rethink : Include Emitted power and also in BaseEnergyModel
    calcIntensityRecord[9] = receivedPower;
    calcIntensityRecord[10] = 0; // By default, assume the point isn't captured
    calcIntensityRecords.push_back(calcIntensityRecord);
#endif
}

double ImprovedEnergyModel::computeEmittedPower(
    ModelArg const &_args
){
    ImprovedEmittedPowerArgs const & args = static_cast<
        ImprovedEmittedPowerArgs const &
    >(_args);
    return EnergyMaths::calcSubrayWiseEmittedPower(
        args.averagePower_w,
        args.targetRange,
        args.beamSampleQuality,
        args.subrayRadiusStep,
        args.numSubrays
    );
}

double ImprovedEnergyModel::computeTargetArea(
    ModelArg const &_args
#if DATA_ANALYTICS >=2
   ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
){
    ImprovedTargetAreaArgs const & args = static_cast<
        ImprovedTargetAreaArgs const &
    >(_args);
    double const rmax = args.targetRange * args.deviceBeamDivergence_rad / 2.0;
    double const dr = rmax / args.beamSampleQuality;
    double const ri = rmax - (
        args.beamSampleQuality - args.subrayRadiusStep - 1.0
    ) * dr;
    double const rbeforei =  (args.subrayRadiusStep == 0) ? 0.0 : ri - dr;
#if DATA_ANALYTICS >= 2
    std::vector<double> calcIntensityRecord(
        11, std::numeric_limits<double>::quiet_NaN()
    );
    calcIntensityRecord[6] = ri;
    calcIntensityRecords.push_back(calcIntensityRecord);
#endif
    // TODO Rethink : Divide by number of subrays at current ring
    double const numSubraysAtRing = (args.subrayRadiusStep == 0) ? 1.0 :
        args.subrayRadiusStep*6.0;  // TODO Rethink : Use a more reliable alternative ?
    return (ri*ri - rbeforei*rbeforei)*M_PI/numSubraysAtRing;
}
