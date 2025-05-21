#pragma once
#include <maths/model/BaseEnergyModel.h>
#include <vector>

// ***  ARGUMENT CLASSES  *** //
// ************************** //
class ImprovedReceivedPowerArgs : public ModelArg
{
public:
  double const targetRange;
  double const incidenceAngle_rad;
  Material const& material;
  int const subrayRadiusStep;
  ImprovedReceivedPowerArgs(double const targetRange,
                            double const incidenceAngle_rad,
                            Material const& material,
                            int const subrayRadiusStep)
    : targetRange(targetRange)
    , incidenceAngle_rad(incidenceAngle_rad)
    , material(material)
    , subrayRadiusStep(subrayRadiusStep)
  {
  }
};

class ImprovedEmittedPowerArgs : public ModelArg
{
public:
  double const targetRange;
  double const targetRangeSquared;
  double const rangeMin;
  int const subrayRadiusStep;
  ImprovedEmittedPowerArgs(double const targetRange,
                           double const targetRangeSquared,
                           double const rangeMin,
                           int const subrayRadiusStep)
    : targetRange(targetRange)
    , targetRangeSquared(targetRangeSquared)
    , rangeMin(rangeMin)
    , subrayRadiusStep(subrayRadiusStep)
  {
  }
};

class ImprovedTargetAreaArgs : public ModelArg
{
public:
  double const targetRangeSquared;
  int const subrayRadiusStep;
  ImprovedTargetAreaArgs(double const targetRangeSquared,
                         int const subrayRadiusStep)
    : targetRangeSquared(targetRangeSquared)
    , subrayRadiusStep(subrayRadiusStep)
  {
  }
};

// ***  IMPROVED ENERGY MODEL CLASS  *** //
// ************************************* //
/**
 * @author Alberto M. Esmoris Pena
 *
 * @brief Class providing an improved energy model with better energy
 *  distribution modeling.
 */
class ImprovedEnergyModel : public BaseEnergyModel
{
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
   * @brief The values of ImprovedEnergyModel::radii but squared.
   */
  std::vector<double> radiiSquared;
  /**
   * @brief The values of ImprovedEnergyModel::radiiSquared but multiplied
   * by \f$-2\f$.
   */
  std::vector<double> negRadiiSquaredx2;
  /**
   * @brief Precomputed squared of beam waist radius:
   *
   * \f[
   *  w_0^2 = \left(\frac{\mathrm{BQ} \lambda}{\pi \varphi_*}\right)^2
   * \f]
   *
   * Where \f$\varphi_*\f$ is the device's beam divergence,
   * \f$\lambda\f$ is the wavelength (in meters), and \f$\mathrm{BQ}\f$
   * the beam quality factor.
   */
  double const w0Squared;
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
  /**
   * @brief Precompute part of the squared \f$\Omega\f$ term, more
   *  concretely:
   *
   * \f[
   *  \left(\frac{\lambda}{\pi w_0^2}\right)^2
   * \f]
   *
   * Where \f$w_0\f$ is the beam waist radius, and \f$\lambda\f$ is the
   *  wavelength (in meters).
   */
  double const omegaCacheSquared;
  /**
   * @brief Precompute part of the target area, more concretely:
   *
   * \f[
   *  \frac{\pi}{n_sr}
   * \f]
   *
   * Where \f$n_sr\f$ is the number of subrays.
   */
  std::vector<double> targetAreaCache;
  /**
   * @brief Precompute the expression involving the device's constants
   * to speedup the subray-wise emitted power computation such that:
   *
   * \f[
   *  \frac{\pi P_{T} w_0^2}{2 n_{sr}}
   * \f]
   *
   * Where \f$P_{T}\f$ is the total power, \f$w_0\f$ is the beam waist
   *  radius, and \f$n_{sr}\f$ is the number of subrays.
   */
  std::vector<double> deviceConstantExpression;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @see EnergyModel::EnergyModel
   */
  ImprovedEnergyModel(ScanningDevice const& sd);

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
  double computeIntensity(double const incidenceAngle,
                          double const targetRange,
                          Material const& mat,
                          int const subrayRadiusStep
#if DATA_ANALYTICS >= 2
                          ,
                          std::vector<std::vector<double>>& calcIntensityRecords
#endif
                          ) override;
  /**
   * @brief Compute the received power \f$P_r\f$.
   *
   * \f[
   *  P_r = \frac{
   *      P_e D_r^2 \eta_{\mathrm{sys}} \eta_{\mathrm{atm}} \sigma
   *  }{
   *      4 \pi R^4 \varphi
   *  }
   * \f]
   *
   * Where:
   * <ol>
   *  <li>\f$P_e\f$ is the emitted power</li>
   *  <li>\f$D_r\f$ is the diameter of the receiver aperture</li>
   *  <li>\f$\eta_{\mathrm{sys}}\f$</li> is the system's efficiency</li>
   *  <li>\f$\eta_{\mathrm{atm}}\f$</li> is the atmospheric efficiency</li>
   *  <li>\f$\sigma\f$ is the cross-section\f$</li>
   *  <li>\f$R\f$ is the range</li>
   *  <li>\f$\varphi\f$ is the subray's beam divergence</li>
   * </ol>
   *
   * @return The received power \f$P_r\f$
   * @see EnergyModel::computeReceivedPower
   * @see ImprovedReceivedPowerArgs
   */
  double computeReceivedPower(
    ModelArg const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
    ) override;
  /**
   * @brief Compute the emitted power \f$P_e\f$.
   *
   * \f[
   *  P_e = \frac{
   *      \pi w_0^2
   *  }{
   *      2 n_{\mathrm{sr}}
   *  }
   *  \biggl[
   *      \exp\left({
   *          - \frac{2 r_{\mathrm{inner}}^2}{w^2}
   *      }\right) -
   *      \exp\left({
   *          - \frac{2 r_{\mathrm{outer}}^2}{w^2}
   *      }\right)
   *  \biggr)]
   *  P_T
   * \f]
   *
   * Where:
   * <ol>
   * <li>\f$w_0\f$ is the beam waist radius</li>
   * <li>\f$n_{\mathrm{sr}}\f$ is the number of subrays at current ring</li>
   * <li>\f$r_{\mathrm{inner}}\f$ is the radius of the inner ring</li>
   * <li>\f$r_{\mathrm{outer}}\f$ is the radius of the outer ring</li>
   * <li>\f$w\f$ is the beam radius for the current range</li>
   * <li>\f$P_T\f$ is the total power reversed following Carlsson et al 2001
   * "Signature simulation and signal analysis for 3-D laser radar" equation
   * 2.3</li>
   * </ol>
   *
   * @return The emitted power \f$P_e\f$.
   * @see EnergyModel::computeEmittedPower
   */
  double computeEmittedPower(ModelArg const& args) override;
  /**
   * @brief Compute the target area \f$A\f$.
   *
   * \f[
   *  A = \pi \frac{
   *      r_{\mathrm{outer}^2 - r_{\mathrm{inner}}^2}
   *  }{
   *      n_{\mathrm{sr}}
   *  }
   * \f]
   *
   * Where:
   * <ol>
   * <li>\f$n_{\mathrm{sr}}\f$ is the number of subrays at current ring</li>
   * <li>\f$r_{\mathrm{inner}}\f$ is the radius of the inner ring</li>
   * <li>\f$r_{\mathrm{outer}}\f$ is the radius of the outer ring</li>
   * </ol>
   *
   * @return The target area \f$A\f$.
   * @see EnergyModel::computeTargetArea
   */
  double computeTargetArea(
    ModelArg const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
    ) override;
};
