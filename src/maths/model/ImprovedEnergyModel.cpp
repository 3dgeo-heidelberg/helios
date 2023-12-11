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
        args.wavelength_m,
        args.rangeMin,
        args.targetRange,
        args.deviceBeamDivergence_rad,
        args.beamWaistRadius,
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
            args.subrayRadiusStep,
            args.numSubrays
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
    );
#if DATA_ANALYTICS >= 2
    std::vector<double> & calcIntensityRecord = calcIntensityRecords.back();
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

double ImprovedEnergyModel::computeEmittedPower(
    ModelArg const &_args
){
    ImprovedEmittedPowerArgs const & args = static_cast<
        ImprovedEmittedPowerArgs const &
    >(_args);
    // TODO Rethink : Review
    // TODO Rethink : Radius and prevRadius are computed twice
    double const angle = args.deviceBeamDivergence_rad/2.0 * (
        args.subrayRadiusStep / (args.beamSampleQuality-0.5)
    );
    double const prevAngle = (args.subrayRadiusStep == 0.0) ? 0.0 :
        args.deviceBeamDivergence_rad / 2.0 * (
            (args.subrayRadiusStep-1.0) / (args.beamSampleQuality-0.5)
    );
    double const radius = angle + args.deviceBeamDivergence_rad/2.0 * (
        0.5 / (args.beamSampleQuality-0.5)
    );
    double const prevRadius = (args.subrayRadiusStep == 0.0) ? 0.0 :
        prevAngle + args.deviceBeamDivergence_rad/2.0 * (
        0.5 / (args.beamSampleQuality-0.5)
    );
    double const w = args.wavelength_m*args.wavelength_m * (
        args.rangeMin*args.rangeMin + args.targetRange*args.targetRange
    ) / (
        M_PI*args.beamWaistRadius*args.beamWaistRadius
    );
    return EnergyMaths::calcSubrayWiseEmittedPower(
        args.averagePower_w,
        w,
        radius,
        prevRadius,
        args.numSubrays
    );
}

double ImprovedEnergyModel::computeTargetArea(
    ModelArg const &_args
#if DATA_ANALYTICS >=2
   ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
){
    // Once for target area and once for emitted power
    ImprovedTargetAreaArgs const & args = static_cast<
        ImprovedTargetAreaArgs const &
    >(_args);
    // TODO Rethink : Review
    // TODO Rethink : Radius and prevRadius are computed twice
    double const angle = args.deviceBeamDivergence_rad/2.0 * (
        args.subrayRadiusStep / (args.beamSampleQuality-0.5)
    );
    double const prevAngle = (args.subrayRadiusStep == 0.0) ? 0.0 :
        args.deviceBeamDivergence_rad / 2.0 * (
        (args.subrayRadiusStep-1.0) / (args.beamSampleQuality-0.5)
    );
    double const radius = angle + args.deviceBeamDivergence_rad/2.0 * (
        0.5 / (args.beamSampleQuality-0.5)
    );
    double const prevRadius = (args.subrayRadiusStep == 0.0) ? 0.0 :
        prevAngle + args.deviceBeamDivergence_rad/2.0 * (
        0.5 / (args.beamSampleQuality-0.5)
    );
    double const radius_m = radius * args.targetRange;
    double const prevRadius_m = (args.subrayRadiusStep == 0) ? 0.0 :
        prevRadius * args.targetRange;
#if DATA_ANALYTICS >= 2
    std::vector<double> calcIntensityRecord(
        13, std::numeric_limits<double>::quiet_NaN()
    );
    calcIntensityRecord[6] = radius_m;
    calcIntensityRecords.push_back(calcIntensityRecord);
#endif
    return M_PI * (radius_m*radius_m - prevRadius_m*prevRadius_m)
        / args.numSubrays;
}
