#pragma once

#include <cmath>
#include <maths/MathConstants.h>
#include <scene/Material.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Some common mathematical operations concerning energy.
 */
class EnergyMaths
{
private:
  // ***  STATIC CLASS  *** //
  // ********************** //
  EnergyMaths() {};
  virtual ~EnergyMaths() = 0;

public:
  // ***  EMITTED / RECEIVED POWER  *** //
  // ********************************** //
  /**
   * @brief Compute the emitted power for a subray such that the sum of the
   *  emitted energy by each subray matches the emitted energy when only a
   *  single ray is used.
   *
   *
   * \f[
   *  P_e = \frac{\pi w_0^2 I_0'}{2n_{sr}} \Biggl(
   *      \exp\biggl[
   *          - \frac{2 r_{i-1}^2}{w^2}
   *      \biggr]
   *      - \exp\biggl[
   *          - \frac{2 r_i^2}{w^2}
   *      \biggr]
   *  \Biggr)
   * \f]
   *
   * @param reversedI0 The reversed average power of the device \f$I_0'\f$
   *  as in Carlsson 2001, Signature simulation and signal analysis for 3D
   *  laser radar, equation 2.3.
   * @param w Let \f$\lambda\f$ be the wavelength in meters,
   *  \f$w_0\f$ be the beam waist radius,
   *  \f$R_0\f$ the minimum range,
   *  \f$R\f$ the range, and
   *  \f$w_0\f$ the beam waist radius
   *  so \f$w\f$ can be defined as:
   *
   * \f[
   * w = w_0 \sqrt{
   *  \left(\dfrac{\lambda R}{\pi w_0^2}\right)^2 +
   *  \left(1 - \dfrac{R}{R_0}\right)^2
   * }
   * \f]
   *
   * @param radius The radius of the ring to which the current subray
   *  belongs \f$r_i\f$ (also outer radius).
   * @param prevRadius the radius of the previous ring, i.e.,
   *  the immediately smaller one \f$r_{i-1}\f$. The previous radius for the
   *  first ring is zero (also inner radius).
   * @param numSubrays The number of subrays in the elliptical footprint
   *  approximation \f$n_{sr}\f$.
   * @return The emitted power for the corresponding subray.
   */
  static double calcSubrayWiseEmittedPower(double const reversedI0,
                                           double const w0,
                                           double const w,
                                           double const radius,
                                           double const prevRadius,
                                           double const numSubrays);

  /**
   * @brief The EnergyMaths::calcSubrayWiseEmittedPower assuming some terms
   *  are given already squared, and the constants defining a device have
   *  already been precomputed so there is no need for those operations.
   * @see EnergyMaths::calcSubrayWiseEmittedPower
   */
  static double calcSubrayWiseEmittedPowerFast(
    double const deviceConstantExpression,
    double const wSquared,
    double const radiusSquared,
    double const prevRadiusSquared);

  /**
   * @brief Solve the laser radar equation
   * @param Pe The emitted power
   * @param Dr2 Squared receiver diameter
   * @param R Target range
   * @param targetArea The target area for the subray
   * @param etaSys Efficiency of scanning device
   * @param etaAtm Atmospheric factor
   * @param sigma Cross section between target area and incidence angle
   * @return Calculated received power
   * @see EnergyModel
   */
  static double calcReceivedPowerFast(double const Pe,
                                      double const Dr2,
                                      double const denom,
                                      double const etaSys,
                                      double const etaAtm,
                                      double const sigma);

  // ***  ATMOSPHERIC STUFF  *** //
  // *************************** //
  /**
   * @brief Compute the atmospheric factor \f$\eta_a\f$, understood as the
   *  energy left after attenuation by air partciles in range \f$[0, 1]\f$
   *
   * \f[
   *  \eta_a = \exp\left( -2 R a_e \right)
   * \f]
   *
   * @param R The target range \f$R\f$
   * @param ae The atmospheric extinction \f$a_e\f$
   * @return The atmospheric factor \f$\eta_a\f$
   */
  static double calcAtmosphericFactor(double const R, double const ae);

  // ***  CROSS-SECTION  *** //
  // *********************** //
  /**
   * @brief Compute cross section
   *
   * \f[
   *  C_{S} = 4{\pi} \cdot f \cdot A_{lf}
   * \f]
   *
   * <br/>
   * Paper DOI: 10.1016/j.isprsjprs.2010.06.007
   *
   * @return Cross section
   * @see computeBRDF
   */
  static double calcCrossSection(double const f, double const Alf);

  // ***  LIGHTING  *** //
  // ****************** //
  /**
   * @brief Compute the material's angular reflectance response using its BRDF.
   *
   * For Phong materials, this multiplies phongBRDF by the reflectance and
   * the cosine of the incidence angle. Lambertian materials return
   * reflectance times cosine; direction-independent materials return
   * reflectance alone.
   * @param mat The material specification.
   * @param incidenceAngle The incidence angle.
   * @return The reflectance response including the incidence cosine where
   * applicable.
   */
  static double computeBRDF(Material const& mat, double const incidenceAngle);
  /**
   * @brief Compute the Phong model
   *
   * <br/>
   * Paper title: NORMALIZATION OF LIDAR INTENSITY DATA BASED ON RANGE AND
   *  SURFACE INCIDENCE ANGLE
   * <br/>
   * Paper authors: B. Jutzi, H. Gross
   *
   * Mathematically the Phong model is described by the equation below. In
   *  this equation, \f$\varphi\f$ is the incidence angle, \f$K_s\f$ is the
   *  specularity scalar, and \f$N_s\f$ is the specular exponent. Note the
   *  specularity scalar is determined from the specular components as
   *  defined in Material::setSpecularity
   *
   * \f[
   *  \mathrm{BRDF}_{\mathrm{PHONG}} = \bigl(1-K_s\bigr) +
   *      \frac{K_s \lvert\cos(2\varphi)\rvert^{N_s}}{\cos(\varphi)}
   * \f]
   *
   * computeBRDF multiplies this factor by the reflectance \f$\rho\f$
   * and the incidence cosine to obtain the angular reflectance response:
   *
   * \f[
   *  f = \rho \mathrm{BRDF}_{\mathrm{PHONG}} \cos(\varphi) = \rho \biggl(
   *      \bigl(1-K_s\bigr) \cos(\varphi) +
   *      K_s \lvert\cos(2\varphi)\rvert^{N_s} \biggr)
   * \f]
   * This helper divides by the incidence cosine. Callers must apply that
   * cosine as computeBRDF does; the standalone factor is singular at grazing
   * incidence (cosine zero). Refactors must preserve this caller constraint.
   */
  static double phongBRDF(double const incidenceAngle,
                          double const targetSpecularity,
                          double const targetSpecularExponent);
  /**
   * @brief The EnergyMaths::phongBRDF function assuming the cosine of the
   * incidence angle is precomputed, thus it is expected to be faster.
   * The supplied cosine must match incidenceAngle and be nonzero. The caller
   * must apply it to the result as documented for phongBRDF.
   * @see EnergyMaths::phongBRDF
   */
  static double phongBRDFFast(double const incidenceAngle,
                              double const cosIncidenceAngle,
                              double const targetSpecularity,
                              double const targetSpecularExponent);
};
