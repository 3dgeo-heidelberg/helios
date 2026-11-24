#pragma once

#include <noise/NoiseSource.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <vector>

/**
 * @brief Leaf angle distribution (LAD) of a turbid medium.
 *
 * The distribution is given as the probability density \f$g_L(\theta_L)\f$
 * of the leaf inclination \f$\theta_L \in [0, \pi/2]\f$, i.e., the angle
 * between leaf normal and zenith, normalized so that
 * \f$\int_0^{\pi/2} g_L(\theta_L)\,d\theta_L = 1\f$. Leaf azimuths are
 * assumed to be uniformly distributed.
 *
 * On construction, \f$g_L\f$ is tabulated on a uniform grid and two tables
 * are derived from it:
 *
 * <ul>
 *  <li>The G-function (Ross, 1981), i.e., the mean projection of unit leaf
 *  area onto the plane perpendicular to the beam direction \f$\Omega'\f$:
 *  \f[
 *    G(\zeta) = \int_0^{\pi/2} g_L(\theta_L)\,
 *      \psi(\zeta, \theta_L)\,d\theta_L
 *  \f]
 *  where \f$\zeta\f$ is the beam zenith angle and \f$\psi\f$ the mean of
 *  \f$|\Omega' \cdot \Omega_L|\f$ over leaf azimuth. The extinction
 *  coefficient of a voxel with plant area density \f$u_L\f$ is
 *  \f$\kappa = u_L\,G(\zeta)\f$.</li>
 *  <li>The cumulative distribution of \f$g_L\f$, used to sample the
 *  orientation of the leaf hit by a beam with probability proportional to
 *  \f$g_L(\Omega_L)\,|\Omega' \cdot \Omega_L|\f$.</li>
 * </ul>
 */
class LeafAngleDistribution
{
public:
  // ***  CONSTANTS  *** //
  // ******************* //
  /**
   * @brief Number of leaf inclination bins used to tabulate \f$g_L\f$
   */
  static constexpr std::size_t INCLINATION_BINS = 900;
  /**
   * @brief Number of beam zenith angles in \f$[0, \pi/2]\f$ at which
   * \f$G\f$ is tabulated
   */
  static constexpr std::size_t G_SAMPLES = 901;
  /**
   * @brief Maximum number of rejection sampling attempts per sample
   */
  static constexpr std::size_t MAX_SAMPLING_ATTEMPTS = 100000;

  // ***  CONSTRUCTION  *** //
  // ********************** //
  /**
   * @brief Build a leaf angle distribution from tabulated values.
   *
   * The values are linearly interpolated onto the internal inclination grid
   * and normalized, so they do not need to be normalized on input.
   *
   * @param inclinations_rad Strictly increasing leaf inclinations in
   * \f$[0, \pi/2]\f$ (at least two values)
   * @param density Non-negative \f$g_L\f$ values, one per inclination, not
   * all zero
   * @param name Name of the distribution
   */
  LeafAngleDistribution(std::vector<double> const& inclinations_rad,
                        std::vector<double> const& density,
                        std::string name = "custom");

  /**
   * @brief Build one of the named distributions of de Wit (1965):
   * planophile, erectophile, plagiophile, extremophile, spherical,
   * uniform. Case insensitive.
   * @throws HeliosException for an unknown name
   */
  static LeafAngleDistribution fromName(std::string const& name);

  /**
   * @brief Load a custom distribution from a text file.
   *
   * Each line holds a leaf inclination in degrees and its \f$g_L\f$ value,
   * separated by a comma. Empty lines and lines starting with '#' are
   * ignored.
   *
   * @throws HeliosException if the file cannot be read or is invalid
   */
  static LeafAngleDistribution fromFile(std::string const& path);

  /**
   * @brief Check whether a name refers to one of the named distributions
   */
  static bool isNamedType(std::string const& name);

  // ***  METHODS  *** //
  // ***************** //
  /**
   * @brief G-function for a beam direction
   * @param direction Beam direction, does not need to be normalized
   */
  double G(glm::dvec3 const& direction) const;

  /**
   * @brief G-function for a beam zenith angle
   * @param zenith_rad Beam zenith angle in \f$[0, \pi]\f$
   */
  double G(double zenith_rad) const;

  /**
   * @brief Sample the incidence angle between a beam and the leaf it hits.
   *
   * Draws a leaf inclination from \f$g_L\f$ and a uniform leaf azimuth, and
   * accepts the resulting normal \f$\Omega_L\f$ with probability
   * \f$|\Omega' \cdot \Omega_L|\f$. Uses three values of the noise source
   * per attempt. The mean acceptance rate is \f$G(\Omega')\f$.
   *
   * @param direction Beam direction, does not need to be normalized
   * @param uniform Noise source returning uniform values in \f$[0, 1]\f$
   * @return Incidence angle \f$\alpha = \arccos|\Omega' \cdot \Omega_L|\f$
   * in \f$[0, \pi/2]\f$
   * @throws HeliosException if no sample is accepted after
   * MAX_SAMPLING_ATTEMPTS attempts
   */
  double sampleIncidenceAngle(glm::dvec3 const& direction,
                              NoiseSource<double>& uniform) const;

  /**
   * @brief Mean of \f$|\Omega' \cdot \Omega_L|\f$ over leaf azimuth (Ross,
   * 1981)
   * @param zenith_rad Beam zenith angle in \f$[0, \pi]\f$
   * @param inclination_rad Leaf inclination in \f$[0, \pi/2]\f$
   */
  static double psi(double zenith_rad, double inclination_rad);

  // ***  GETTERS  *** //
  // ***************** //
  std::string const& getName() const { return name; }
  /**
   * @brief Centres of the leaf inclination bins (radians)
   */
  std::vector<double> const& getInclinations() const { return inclinations; }
  /**
   * @brief Normalized \f$g_L\f$ at the bin centres
   */
  std::vector<double> const& getDensity() const { return density; }

private:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  std::string name;
  std::vector<double> inclinations;
  std::vector<double> density;
  /**
   * @brief Cumulative probability at the upper edge of each bin
   */
  std::vector<double> cdf;
  /**
   * @brief \f$G\f$ at G_SAMPLES equally spaced zenith angles in
   * \f$[0, \pi/2]\f$
   */
  std::vector<double> gTable;

  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Normalize \f$g_L\f$ given at the bin centres and build the CDF
   * and G tables
   */
  void build(std::vector<double> binDensity);
  static double binWidth();
};
