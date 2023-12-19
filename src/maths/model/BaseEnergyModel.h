#pragma once

#include <maths/model/EnergyModel.h>
#include <maths/model/ModelArg.h>
#include <scene/Material.h>

// ***  ARGUMENT CLASSES  *** //
// ************************** //
class BaseReceivedPowerArgs : public ModelArg {
public:
    double const incidenceAngle_rad;
    double const targetRange;
    Material const &material;
    double const subrayRadius;
    BaseReceivedPowerArgs(
        double const incidenceAngle_rad,
        double const targetRange,
        Material const &material,
        double const subrayRadius
    ) :
        incidenceAngle_rad(incidenceAngle_rad),
        targetRange(targetRange),
        material(material),
        subrayRadius(subrayRadius)
    {}
};

class BaseEmittedPowerArgs : public ModelArg {
public:
    double const averagePower_w;
    double const wavelength_m;
    double const targetRange;
    double const rangeMin;
    double const subrayRadius;
    double const beamWaistRadius;
    BaseEmittedPowerArgs(
        double const averagePower_w,
        double const wavelength_m,
        double const targetRange,
        double const rangeMin,
        double const subrayRadius,
        double const beamWaistRadius
    ) :
        averagePower_w(averagePower_w),
        wavelength_m(wavelength_m),
        targetRange(targetRange),
        rangeMin(rangeMin),
        subrayRadius(subrayRadius),
        beamWaistRadius(beamWaistRadius)
    {}
};

class BaseTargetAreaArgs : public ModelArg {
public:
    double const squaredRange;
    BaseTargetAreaArgs(
        double const squaredRange
    ) :
        squaredRange(squaredRange)
    {}
};

class BaseCrossSectionArgs : public ModelArg {
public:
    Material const &material;
    double const bdrf; // Bidirectional reflectance function
    double const targetArea;
    BaseCrossSectionArgs(
        Material const &material,
        double const bdrf,
        double const targetArea
    ) :
        material(material),
        bdrf(bdrf),
        targetArea(targetArea)
    {}
};


// ***  ENERGY MODEL CLASS  *** //
// **************************** //
/**
 * @author Alberto M. Esmoris Pena
 *
 * @brief Class providing a baseline energy model.
 */
class BaseEnergyModel : public EnergyModel {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Precomputed value for the target area computation.
     *
     * \f[
     *  \frac{\pi \varphi_*}{4 n_s}
     * \f]
     */
    double targetAreaCache;
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @see EnergyModel::EnergyModel
     */
    BaseEnergyModel(ScanningDevice const &sd);

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
    /**
     * @brief Compute the received power \f$P_r\f$.
     *
     * \f[
     *  P_r = \frac{
     *      P_\mu D_r^2 \eta_{\mathrm{sys}} \sigma
     *  }{
     *      4 \pi R^4 \varphi_*^2 \exp\left({
     *          2 R a_e + \frac{
     *              2\pi^2r^2w_0^2
     *          }{
     *              \lambda^2(R_0^2 + R^2)
     *          }
     *      }\right)
     *  }
     *  10^9
     * \f]
     *
     * Where:
     * <ol>
     * <li>\f$P_\mu\f$ is the average power</li>
     * <li>\f$D_r^2\f$ is the squared aperture diameter</li>
     * <li>\f$\eta_{\mathrm{sys}}\f$ is the optical efficiency</li>
     * <li>\f$\sigma\f$ is the cross-section</li>
     * <li>\f$R\f$ is the target range</li>
     * <li>\f$\varphi_*^2\f$ is the squared device's beam divergence (not the
     *  subray's beam divergence)</li>
     * <li>\f$r\f$ is the subray's radius</li>
     * <li>\f$w_0\f$ is the beam waist radius</li>
     * <li>\f$\lambda\f$ is the wavelength</li>
     * <li>\f$R_0\f$ is the device's min range</li>
     * </ol>
     *
     * @return The received power \f$P_r\f$.
     * @see EnergyModel::computeReceivedPower
     * @see BaseReceivedPowerArgs
     */
    double computeReceivedPower(
        ModelArg const & args
#if DATA_ANALYTICS >=2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
    ) override;
    /**
     * @brief Compute the emitted power \f$P_e\f$.
     *
     * \f[
     *  P_e = P_\mu \exp\left(\frac{
     *          -2 r^2
     *      }{
     *          w^2
     *      }\right)
     * \f]
     *
     * with
     *
     * \f[
     *  w^2 = \frac{
     *      \lambda^2 (R_0^2 + R^2)
     *  }{
     *      \pi^2 w_0^2
     *  }
     * \f]
     *
     * Where:
     * <ol>
     * <li>\f$P_\mu\f$ is the average power</li>
     * <li>\f$R_0\f$ is the device's min range</li>
     * <li>\f$R\f$ is the target range</li>
     * <li>\f$r\f$ is the subray's radius</li>
     * <li>\f$w_0\f$ is the beam waist radius</li>
     * <li>\f$\lambda\f$ is the wavelength</li>
     * </ol>
     *
     * @return The emitted power \f$P_e\f$.
     * @see EnergyModel::computeEmittedPower
     */
    double computeEmittedPower(ModelArg const & args) override;
    /**
     * @brief Compute the target area \f$A\f$.
     *
     * \f[
     *  A = \frac{
     *      \pi R^2 \varphi_*^2
     *  }{
     *      4 n_{sr}
     *  }
     * \f]
     *
     * Where
     * <ol>
     * <li>\f$R\f$ is the target range</li>
     * <li>\f$\varphi_*^2\f$ is the squared device's beam divergence (not the
     *  subray's beam divergence)</li>
     * <li>\f$n_{sr}\f$ is the number of subrays per ray according to the
     *  discrete model</li>
     * </ol>
     *
     * @return The target area \f$A\f$.
     * @see EnergyModel::computeTargetArea
     */
    double computeTargetArea(
        ModelArg const & args
#if DATA_ANALYTICS >=2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
    ) override;
    /**
     * @brief Compute the cross section \f$\sigma\f$.
     *
     * \f[
     *  \sigma = 4 \pi A \cos(\theta) \mathrm{BDRF}
     * \f]
     *
     * Where
     * <ol>
     * <li>\f$A\f$ is the target area</li>
     * <li>\f$\theta\f$ is the incidence angle</li>
     * <li>\f$\mathrm{BDRF}\f$ is the bidirectional reflectance function</li>
     * </ol>
     *
     * Note that the \f$\mathrm{BDRF}\f$ will change depending on the light
     * model such that:
     *
     * <ol>
     * <li>Phong: \f[
     *  \mathrm{BDRF} = \rho \left[(1-\mathrm{spec}) + \mathrm{spec} \frac{
     *      \cos(2\theta)^{N_s}
     *  }{
     *      \cos(\theta)
     *  }\right]
     * \f]</li>
     * <li>Lambertian: \f[
     *  \mathrm{BDRF} = \rho
     * \f]</li>
     * <li>Direction-independent: \f[
     *  \mathrm{BDRF} = \frac{\rho}{\cos(\theta)}
     * \f]</li>
     * </ol>
     *
     * Where
     * <ol>
     * <li>\f$\rho\f$ is the material's reflectance</li>
     * <li>\f$\mathrm{spec}\f$ is the specular component</li>
     * <li>\f$\mathrm{N_s}\f$ is the specular exponent</li>
     * </ol>
     *
     * @return The target area \f$\sigma\f$.
     * @see EnergyModel::computeCrossSection
     */
    double computeCrossSection(ModelArg const & args) override;

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
    static BaseReceivedPowerArgs extractArgumentsFromScanningDevice(
        ScanningDevice const &sd,
        double const incidenceAngle,
        double const targetRange,
        Material const &mat,
        double const radius,
        int const subrayRadiusStep
    );

};