#include <ImprovedEnergyModel.h>
#include <maths/EnergyMaths.h>
#include <scanner/ScanningDevice.h>
#include <scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ImprovedEnergyModel::ImprovedEnergyModel(ScanningDevice const &sd) :
    BaseEnergyModel(sd),
    radii(sd.FWF_settings.beamSampleQuality+1),
    w0(sd.beamQuality * sd.wavelength_m / (M_PI*sd.beamDivergence_rad)),
    omegaCache(sd.wavelength_m/(M_PI*w0*w0)),
    totPower(2*sd.averagePower_w / (M_PI*w0*w0))
{
    // Cached angles
    int const BSQ = sd.FWF_settings.beamSampleQuality;
    radii[0] = 0.0;
    for(int i = 0 ; i < BSQ ; ++i){
        radii[i+1] = sd.beamDivergence_rad*(
            sd.cached_subrayRadiusStep[i]+0.5
        ) / (2 * (BSQ-0.5));
    }
    // TODO Rethink : Implement
}


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
    double prevRadius = radii[int(args.subrayRadiusStep)];
    double const radius = radii[int(args.subrayRadiusStep)+1];
    double const Omega0 = 1 - args.targetRange/args.rangeMin;
    double const Omega = args.targetRange * omegaCache;
    double const w = w0 * std::sqrt(Omega0*Omega0 + Omega*Omega);
    return EnergyMaths::calcSubrayWiseEmittedPower(
        totPower,
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
    double const prevRadius = radii[int(args.subrayRadiusStep)];
    double const radius = radii[int(args.subrayRadiusStep)+1];
    double const radius_m = radius * args.targetRange;
    double const prevRadius_m = prevRadius * args.targetRange;
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
