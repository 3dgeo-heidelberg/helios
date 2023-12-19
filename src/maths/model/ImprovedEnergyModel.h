#pragma once
#include <maths/model/BaseEnergyModel.h>

// ***  ARGUMENT CLASSES  *** //
// ************************** //
class ImprovedReceivedPowerArgs : public ModelArg {
public:
    double const averagePower_w;
    double const wavelength_m;
    double const rangeMin;
    double const targetRange;
    double const atmosphericExtinction;
    double const incidenceAngle_rad;
    double const Dr2;  // Squared aperture diameter
    double const Bt2;  // Squared device's beam divergence (not subray)
    double const efficiency;
    double const numSubrays;
    Material const &material;
    double const deviceBeamDivergence_rad;
    double const beamWaistRadius;
    double const beamSampleQuality;
    double const beamQualityFactor;
    double const subrayRadiusStep;
    ImprovedReceivedPowerArgs(
        double const averagePower_w,
        double const wavelength_m,
        double const rangeMin,
        double const targetRange,
        double const atmosphericExtinction,
        double const incidenceAngle_rad,
        double const Dr2,
        double const Bt2,
        double const efficiency,
        double const numSubrays,
        Material const &material,
        double const deviceBeamDivergence_rad,
        double const beamWaistRadius,
        double const beamSampleQuality,
        double const beamQualityFactor,
        double const subrayRadiusStep
    ) :
        averagePower_w(averagePower_w),
        wavelength_m(wavelength_m),
        rangeMin(rangeMin),
        targetRange(targetRange),
        atmosphericExtinction(atmosphericExtinction),
        incidenceAngle_rad(incidenceAngle_rad),
        Dr2(Dr2),
        Bt2(Bt2),
        efficiency(efficiency),
        numSubrays(numSubrays),
        material(material),
        deviceBeamDivergence_rad(deviceBeamDivergence_rad),
        beamWaistRadius(beamWaistRadius),
        beamSampleQuality(beamSampleQuality),
        beamQualityFactor(beamQualityFactor),
        subrayRadiusStep(subrayRadiusStep)
    {}
};

class ImprovedEmittedPowerArgs : public ModelArg {
public:
    double const averagePower_w;
    double const wavelength_m;
    double const rangeMin;
    double const targetRange;
    double const deviceBeamDivergence_rad;
    double const beamWaistRadius;
    double const numSubrays;
    double const beamSampleQuality;
    double const beamQualityFactor;
    double const subrayRadiusStep;
    ImprovedEmittedPowerArgs(
        double const averagePower_w,
        double const wavelength_m,
        double const rangeMin,
        double const targetRange,
        double const deviceBeamDivergence_rad,
        double const beamWaistRadius,
        double const numSubrays,
        double const beamSampleQuality,
        double const beamQualityFactor,
        double const subrayRadiusStep
    ) :
        averagePower_w(averagePower_w),
        wavelength_m(wavelength_m),
        rangeMin(rangeMin),
        targetRange(targetRange),
        deviceBeamDivergence_rad(deviceBeamDivergence_rad),
        beamWaistRadius(beamWaistRadius),
        numSubrays(numSubrays),
        beamSampleQuality(beamSampleQuality),
        beamQualityFactor(beamQualityFactor),
        subrayRadiusStep(subrayRadiusStep)
    {}
};

class ImprovedTargetAreaArgs : public ModelArg{
public:
    double const targetRange;
    double const deviceBeamDivergence_rad;
    double const beamSampleQuality;
    double const subrayRadiusStep;
    double const numSubrays;
    ImprovedTargetAreaArgs(
        double const targetRange,
        double const deviceBeamDivergence_rad,
        double const beamSampleQuality,
        double const subrayRadiusStep,
        double const numSubrays
    ) :
        targetRange(targetRange),
        deviceBeamDivergence_rad(deviceBeamDivergence_rad),
        beamSampleQuality(beamSampleQuality),
        subrayRadiusStep(subrayRadiusStep),
        numSubrays(numSubrays)
    {}
};

// ***  IMPROVED ENERGY MODEL CLASS  *** //
// ************************************* //
/**
 * @author Alberto M. Esmoris Pena
 *
 * @brief Class providing an improved energy model with better energy
 *  distribution modeling.
 */
class ImprovedEnergyModel : public BaseEnergyModel {
    // ***  ATTRIBUTES  *** //
    // ******************** //

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @see EnergyModel::EnergyModel
     */
    ImprovedEnergyModel(ScanningDevice const &sd) : BaseEnergyModel(sd) {}

    // TODO Rethink : Some arguments as beamSampleQuality might be "cached" in the class apart from the look-up tables?
    // ***  METHODS  *** //
    // ***************** //
    /**
     * @brief Compute the intensity, i.e., the received power from the given
     *  scanning device and input arguments.
     * @return The computed intensity or received power.
     * @see BaseEnergyModel::computeReceivedPower
     * @see BaseEnergyModel::extractArgumentsFromScanningDevice
     * @see EnergyModel::computeIntensity
     */
    double computeIntensity(
        double const incidenceAngle,
        double const targetRange,
        Material const &mat,
        double const radius,
        int const subrayRadiusStep
    );
    // TODO Rethink : Doxygen doc
    double computeReceivedPower(
        ModelArg const & args
#if DATA_ANALYTICS >=2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
    ) override;
    // TODO Rethink : Doxygen doc
    double computeEmittedPower(ModelArg const &args) override;
    // TODO Rethink : Doxygen doc
    double computeTargetArea(
        ModelArg const &args
#if DATA_ANALYTICS >=2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
    ) override;

    // ***  EXTRACT-ARGUMENTS METHODS  *** //
    // *********************************** //
    /**
     * @brief Extract the received power arguments from the given scanning
     *  device.
     * @param sd The scanning device to extract arguments from.
     * @return The extracted received power arguments.
     * @see ScanningDevice
     * @see BaseReceivedPowerArgs
     */
    static ImprovedReceivedPowerArgs extractArgumentsFromScanningDevice(
        ScanningDevice const &sd,
        double const incidenceAngle,
        double const targetRange,
        Material const &mat,
        double const radius,
        int const subrayRadiusStep
    );
};