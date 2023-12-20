#pragma once
#include <maths/model/BaseEnergyModel.h>
#include <vector>

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
    /**
     * @brief Precomptued radii for each subradius step (i.e., ring).
     *
     * radii[0] is 0, and for any \f$i>0\f$ radii[i] is:
     *
     * \f[
     *  \frac{\varphi_* (s+0.5)}{2 (\mathrm{BSQ}-0.5)}
     * \f]
     *
     * Where \f$\varphi_*\f$ is the device's beam divergence,
     * \f$s\f$ is the subray radius step, and \f$\mathrm{BSQ}\f$ is the beam
     * sample quality.
     */
    std::vector<double> radii;
    /**
     * @brief Precomputed beam waist radius:
     *
     * \f[
     *  w_0 = \frac{\mathrm{BQ} \lambda}{\pi \varphi_*}
     * \f]
     *
     * Where \f$\varphi_*\f$ is the device's beam divergence,
     * \f$\lambda\f$ is the wavelength (in meters), and \f$\mathrm{BQ}\f$
     * the beam quality factor.
     */
    double const w0;
    /**
     * @brief Precompute part of the \f$\Omega\f$ term, more concretely:
     *
     * \f[
     *  \frac{\lambda}{\pi w_0^2}
     * \f]
     *
     * Where \f$w_0\f$ is the beam waist radius, and \f$\lambda\f$ is the
     *  wavelength (in meters).
     */
    double const omegaCache;
    /**
     * @brief Precompute the total power from the average power by undoing
     *  Equation 2.3 in Carlsson et al. 2021 (
     *      Signature simulation and signal analysis for 3-D laser radar
     *  ) such that:
     *
     *  \f[
     *      \frac{2 P_{\mu}}{\pi w_0^2}
     *  \f]
     *
     * Where \f$P_{\mu}\f$ is the average power, and \f$w_0\f$ is the beam
     *  waist radius.
     */
    double const totPower;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @see EnergyModel::EnergyModel
     */
    ImprovedEnergyModel(ScanningDevice const &sd);

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