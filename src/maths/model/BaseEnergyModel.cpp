#include <maths/model/BaseEnergyModel.h>
#include <maths/EnergyMaths.h>

// ***  METHODS  *** //
// ***************** //
double BaseEnergyModel::computeReceivedPower(
    ModelArg const & _args
#if DATA_ANALYTICS >=2
    ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
){
    BaseReceivedPowerArgs const & args = static_cast<
        BaseReceivedPowerArgs const&
    >(_args);
    double const targetArea = computeTargetArea(BaseTargetAreaArgs{
        args.targetRange,
        args.Bt2,
        args.numSubrays
    });
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
    double const receivedPower = EnergyMaths::calcReceivedPower(
        args.averagePower_w,
        args.wavelength_m,
        args.targetRange,
        args.rangeMin,
        args.subrayRadius,
        args.beamWaistRadius,
        args.Dr2,
        args.Bt2,
        args.efficiency,
        args.atmosphericExtinction,
        sigma
    ) * 1e09;
#if DATA_ANALYTICS >= 2
    std::vector<double> calcIntensityRecord(
        11, std::numeric_limits<double>::quiet_NaN()
    );
    calcIntensityRecord[3] = args.incidenceAngle_rad;
    calcIntensityRecord[4] = args.targetRange;
    calcIntensityRecord[5] = targetArea;
    calcIntensityRecord[6] = args.radius;
    calcIntensityRecord[7] = bdrf;
    calcIntensityRecord[8] = sigma;
    calcIntensityRecord[9] = receivedPower;
    calcIntensityRecord[10] = 0; // By default, assume the point isn't captured
    calcIntensityRecords.push_back(calcIntensityRecord);
#endif
    return receivedPower;
}

double BaseEnergyModel::computeEmittedPower(
    ModelArg const & _args
){
    BaseEmittedPowerArgs const & args = static_cast<
        BaseEmittedPowerArgs const &
    >(_args);
    return EnergyMaths::calcEmittedPower(
        args.averagePower_w,
        args.wavelength_m,
        args.targetRange,
        args.rangeMin,
        args.subrayRadius,
        args.beamWaistRadius
    );
}

double BaseEnergyModel::computeTargetArea(
    ModelArg const & _args
){
    BaseTargetAreaArgs const & args = static_cast<
        BaseTargetAreaArgs const &
    >(_args);
    return (PI_QUARTER * args.range*args.range * args.Bt2) /
        args.numSubrays;
}

double BaseEnergyModel::computeCrossSection(
    ModelArg const & _args
){
    BaseCrossSectionArgs const & args = static_cast<
        BaseCrossSectionArgs const &
    >(_args);
    return EnergyMaths::calcCrossSection(
        args.bdrf,
        args.targetArea,
        args.incidenceAngle_rad
    );
}
