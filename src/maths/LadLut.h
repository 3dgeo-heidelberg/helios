#pragma once

#include <glm/glm.hpp>
#include <vector>

/**
 * @brief Leaf Angle Distribution Look-Up Table
 *
 * Class representing the look-up table for leaf angle distribution
 */
class LadLut
{
  // *********************** //

public:
  // ***  CONSTANTS  *** //
  // ******************* //
  /**
   * @brief Used to consider decimal precision. Two values will be treated
   * as equal when the difference between them is not greater than eps
   * (\f$\epsilon\f$)
   */
  static const double eps;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief X component of normalized director vector for each record
   */
  std::vector<double> X;
  /**
   * @brief Y component of normalized director vector for each record
   */
  std::vector<double> Y;
  /**
   * @brief Z component of normalized director vector for each record
   */
  std::vector<double> Z;
  /**
   * @brief Function evaluation for each record
   */
  std::vector<double> G;
  /**
   * @brief The angle for each director vector with respect to
   * \f$(0, 0, 1)\f$
   */
  std::vector<double> angles;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief LadLut default constructor
   */
  LadLut() = default;
  LadLut(LadLut& ll);
  virtual ~LadLut() = default;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Compute angles
   * @see LadLut::angles
   */
  void computeAngles();
  /**
   * @see LadLut::computeSigma(double, double, double, double)
   */
  double computeSigma(double padBVTotal, glm::dvec3 direction)
  {
    return computeSigma(padBVTotal, direction.x, direction.y, direction.z);
  }

  /**
   * @brief Compute sigma (\f$\sigma\f$)
   *
   * \f[
   *  \sigma = \frac{p \cdot g}{2{\pi}}
   * \f]
   *
   * @param padBVTotal The PadBVTotal value (\f$p\f$)
   * @param x The x component of normalized director vector
   * @param y The y component of normalized director vector
   * @param z The z component of normalized director vector
   * @return Sigma \f$\sigma\f$
   */
  double computeSigma(double padBVTotal, double x, double y, double z);

  /**
   * @see LadLut::interpolate(double, double, double)
   */
  double interpolate(glm::dvec3 direction)
  {
    return interpolate(direction.x, direction.y, direction.z);
  }
  /**
   * @brief Interpolate \f$g\f$
   *
   * Considering known evaluations, interpolate the value for given
   * direction.
   *
   * First, \f$\hat{u}\f$ and \f$\hat{v}\f$ are the immediately after-before
   * kown director vectors with respect to \f$\hat{w}\f$,
   * which is the one that must be interpolated.
   *
   * Second, \f$a\f$ and \f$b\f$ are the absolute value of angular distance
   * between \f$\hat{u}\f$ and \f$\hat{w}\f$ and between \f$\hat{v}\f$ and
   * \f$\hat{w}\f$ respectively.
   *
   * Therefore, \f$\Delta = a+b\f$.
   *
   * Now weights are defined as \f$\alpha = \frac{b}{\Delta}\f$ and
   * \f$\beta = \frac{a}{\Delta}\f$.
   *
   * And \f$g\f$ can be interpolated:
   * \f[
   * g_{\hat{w}} = {\alpha} g_{\hat{u}} + {\beta} g_{\hat{v}}
   * \f]
   *
   * Finally, in case \f$a\f$ or \f$b\f$ is 0, there is a coincidence
   * with respecct to \f$\hat{w}\f$. In consequence, it is not necessary
   * to compute interpolation as \f$g\f$ is contained in the look-up table.
   *
   * @param x X component of normalized director vector for interpolation
   * @param y Y component of normalized director vector for interpolation
   * @param z Z component of normalized director vector for interpolation
   * @return Interpolated \f$g\f$
   *
   * @see LadLut::findEnvelopmentDirections
   */
  double interpolate(double x, double y, double z);

  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Transform a vector \f$(a,b,c)\f$ from standard domain to a vector
   * \f$(x,y=0,z)\f$ in LadLut domain
   * @param[in] a X component of the vector to transform
   * @param[in] b Y component of the vector to transform
   * @param[in] c Z component of the vector to transform
   * @param[out] x X component of transformed vector
   * @param[out] y Y component of transformed vector
   * @param[out] z Z component of transformed vector
   */
  void transformToLadLutDomain(double a,
                               double b,
                               double c,
                               double& x,
                               double& y,
                               double& z);
  /**
   * @brief Fast verision of transformToLadLutDomain function
   *
   * \f[
   *  \hat{u} = (x, y, z) \longrightarrow
   *      \hat{v} = \left(\sqrt{x^{2}+y^{2}}, 0, z\right)
   * \f]
   *
   * @see LadLut::transformToLadLutDomain
   */
  inline void fastTransformToLadLutDomain(double a,
                                          double b,
                                          double c,
                                          double& x,
                                          double& y,
                                          double& z);
  /**
   * @brief Find the vector immediately after \f$\hat{w}\f$ (\f$\hat{u}\f$)
   * and the one immediately before \f$\hat{w}\f$ (\f$\hat{v}\f$)
   * @param[in] wx X component of w vector
   * @param[in] wy Y component of w vector
   * @param[in] wz Z component of w vector
   * @param[out] uIdx Index of the closet direction after w
   * @param[out] ux X component of the closest direction after w
   * @param[out] uy Y component of the closest direction after w
   * @param[out] uz Z component of the closest direction after w
   * @param[out] vIdx Index of the closest direction before w
   * @param[out] vx X component of the closest direction before w
   * @param[out] vy Y component of the closest direction before w
   * @param[out] vz Z component of the closest direction before w
   * @param[out] a Angular distance between u and w
   * @param[out] b Angular distance between v and w
   */
  void findEnvelopmentDirections(double wx,
                                 double wy,
                                 double wz,
                                 size_t& uIdx,
                                 double& ux,
                                 double& uy,
                                 double& uz,
                                 size_t& vIdx,
                                 double& vx,
                                 double& vy,
                                 double& vz,
                                 double& a,
                                 double& b);
};
