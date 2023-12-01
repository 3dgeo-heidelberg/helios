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
    });
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
    return EnergyMaths::calcReceivedPowerLegacy(
        Pe,
        args.Dr2,
        args.targetRange,
        args.Bt2,
        args.efficiency,
        atmosphericFactor,
        sigma
    );
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
){
    ImprovedTargetAreaArgs const & args = static_cast<
        ImprovedTargetAreaArgs const &
    >(_args);
    double const rmax = args.targetRange * args.deviceBeamDivergence_rad / 2.0;
    double const dr = rmax / args.beamSampleQuality;
    double const ri = rmax - (
        args.beamSampleQuality - args.subrayRadiusStep - 1.0
    ) * dr;
    double const rbeforei =  (args.subrayRadiusStep == 0) ? 0.0 :
        rmax - (args.beamSampleQuality - args.subrayRadiusStep - 2.0) * dr;
    return (ri*ri - rbeforei*rbeforei)*M_PI;
}
