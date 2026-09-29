#pragma once

#include <maths/model/EnergyModelArg.h>
#include <scene/Material.h>
#include <vector>

// ***  FORWARD DECLARATIONS  *** //
// ****************************** //
class ScanningDevice;

/**
 * @author Alberto M. Esmoris Pena
 *
 * @brief Abstract class providing the interface for any energy model.
 */
class EnergyModel
{
private:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The scanning device attached to the energy model
   */
  ScanningDevice const& sd;
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
   * @brief The values of EnergyModel::radii but squared.
   */
  std::vector<double> radiiSquared;
  /**
   * @brief The values of EnergyModel::radiiSquared but multiplied
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
   * @brief Instantiate the EnergyModel such that it is attached to the
   *  given scanning device.
   * @param sd The scanning device attached to the energy model.
   * @see EnergyModel::ScanningDevice
   */
  EnergyModel(ScanningDevice const& sd);

  ~EnergyModel() = default;

  // ***  METHODS  *** //
  // ***************** //
  /**
   * @brief Compute the intensity, i.e., the received power from the given
   *  scanning device and input arguments.
   * @param incidenceAngle The incidence angle (in radians)
   * @param targetRange The raget range (in meters)
   * @param mat The material specification
   * @param radius The subray radius
   * @param subrayRadiusStep The step corresponding to the subray radius
   *  (i.e., ring).
   * @return The computed intensity or received power.
   */
  double computeIntensity(double const incidenceAngle,
                          double const targetRange,
                          Material const& mat,
                          int const subrayRadiusStep
#if DATA_ANALYTICS >= 2
                          ,
                          std::vector<std::vector<double>>& calcIntensityRecords
#endif
  );
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
   * @return The received power \f$P_r\f$.
   */
  double computeReceivedPower(
    ReceivedPowerArgs const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
  );
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
   */
  double computeEmittedPower(EmittedPowerArgs const& args);
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
   */
  double computeTargetArea(
    TargetAreaArgs const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
  );
  /**
   * @brief Compute the cross section \f$\sigma\f$.
   *
   * Note this method must be overriden by any concrete class providing
   * a computable energy model.
   *
   * @return The target area \f$\sigma\f$.
   */
  double computeCrossSection(CrossSectionArgs const& args);
};
