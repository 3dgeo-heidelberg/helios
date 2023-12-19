#include <ImprovedEnergyModel.h>
#include <maths/EnergyMaths.h>
#include <scanner/ScanningDevice.h>
#include <scanner/detector/AbstractDetector.h>

// ***  METHODS  *** //
// ***************** //
double ImprovedEnergyModel::computeIntensity(
    double const incidenceAngle,
    double const targetRange,
    Material const &mat,
    double const radius,
    int const subrayRadiusStep
){
    ImprovedReceivedPowerArgs args = ImprovedEnergyModel::extractArgumentsFromScanningDevice(
        sd,
        incidenceAngle,
        targetRange,
        mat,
        radius,
        subrayRadiusStep
    );
    return computeReceivedPower(args);
}

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
        args.beamQualityFactor,
        args.subrayRadiusStep
    });
    double const atmosphericFactor = EnergyMaths::calcAtmosphericFactor(
        args.targetRange, args.atmosphericExtinction
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
        targetArea
    });
    // --- TODO Rethink : To common impl, consider also BaseEnergyModel
    double const receivedPower = EnergyMaths::calcReceivedPowerImproved(
        Pe,
        args.Dr2,
        args.targetRange,
        targetArea,
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
    // TODO Rethink : Some expressions can be simplified and are unnecessary
    // TODO Rethink : e.g., w is not needed as w^2 is used later on
    // TODO Rethink : Angle and radius stuff
    double const devBeamDiv_rad = args.deviceBeamDivergence_rad;
    double const angle = devBeamDiv_rad/2.0 * (
        args.subrayRadiusStep / (args.beamSampleQuality-0.5)
    );
    double const prevAngle = (args.subrayRadiusStep == 0.0) ? 0.0 :
        devBeamDiv_rad / 2.0 * (
            (args.subrayRadiusStep-1.0) / (args.beamSampleQuality-0.5)
    );
    double const radius = angle + devBeamDiv_rad/2.0 * (
        0.5 / (args.beamSampleQuality-0.5)
    );
    double const prevRadius = (args.subrayRadiusStep == 0.0) ? 0.0 :
        prevAngle + devBeamDiv_rad/2.0 * (
        0.5 / (args.beamSampleQuality-0.5)
    );
    double const w0 = args.beamQualityFactor * args.wavelength_m / (
        M_PI*args.deviceBeamDivergence_rad);
    double const Omega0 = 1 - args.targetRange/args.rangeMin;
    double const Omega = args.wavelength_m*args.targetRange / (M_PI*w0*w0);
    double const w = w0 * std::sqrt(Omega0*Omega0 + Omega*Omega);
    return EnergyMaths::calcSubrayWiseEmittedPower(
        2*args.averagePower_w / (M_PI*w0*w0),  // TODO Rethink : New tot_power
        w0,
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
    // TODO Rethink : Radius and prevRadius are computed twice (see emitted power)
    // TODO Rethink : Angle and radius might be precomputed
    double const devBeamDiv_rad = args.deviceBeamDivergence_rad;
    double const angle = devBeamDiv_rad/2.0 * (
        args.subrayRadiusStep / (args.beamSampleQuality-0.5)
    );
    double const prevAngle = (args.subrayRadiusStep == 0.0) ? 0.0 :
        devBeamDiv_rad / 2.0 * (
        (args.subrayRadiusStep-1.0) / (args.beamSampleQuality-0.5)
    );
    double const radius = angle + devBeamDiv_rad/2.0 * (
        0.5 / (args.beamSampleQuality-0.5)
    );
    double const prevRadius = (args.subrayRadiusStep == 0.0) ? 0.0 :
        prevAngle + devBeamDiv_rad/2.0 * (
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

// ***  EXTRACT-ARGUMENTS METHODS  *** //
// *********************************** //
ImprovedReceivedPowerArgs ImprovedEnergyModel::extractArgumentsFromScanningDevice(
    ScanningDevice const &sd,
    double const incidenceAngle,
    double const targetRange,
    Material const &mat,
    double const radius,
    int const subrayRadiusStep
){
    return ImprovedReceivedPowerArgs(
        sd.averagePower_w,
        sd.wavelength_m,
        sd.detector->cfg_device_rangeMin_m,
        targetRange,
        sd.atmosphericExtinction,
        incidenceAngle,
        sd.cached_Dr2,
        sd.cached_Bt2,
        sd.efficiency,
        (double) ( (subrayRadiusStep==0) ? 1 :
                   (int) (subrayRadiusStep*PI_2) // 2 pi x depth = samples/ring
        ),
        mat,
        sd.beamDivergence_rad,
        sd.beamWaistRadius,
        (double) sd.FWF_settings.beamSampleQuality,
        (double) sd.beamQuality,
        (double) subrayRadiusStep
    );
}
