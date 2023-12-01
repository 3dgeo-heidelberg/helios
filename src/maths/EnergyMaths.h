#pragma once

#include <cmath>
#include <maths/MathConstants.h>
#include <scene/Material.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Some common mathematical operations concerning energy.
 */
class EnergyMaths{
private:
    // ***  STATIC CLASS  *** //
    // ********************** //
    EnergyMaths() {};
    virtual ~EnergyMaths() = 0;

public:
    // ***  EMITTED / RECEIVED POWER  *** //
    // ********************************** //
    /**
     * @brief Compute the space distribution equation to calculate the beam
     *  energy decreasing the further away from the center.
     *
     * \f[
     *  P_e = I_0 \exp\left[- \frac{
     *          2 \pi^2 r^2 w_0^2
     *      }{
     *          \lambda^2 \left(R_0^2 + R^2\right)
     *      }\right]
     * \f]
     *
     * @param I0 The average power
     * @param lambda The wavelength
     * @param R The target range (in meters)
     * @param R0 The minimum range of the device (in meters)
     * @param r The radius
     * @param w0 The beam waist radius
     * @return Calculated emitted power
     */
    static double calcEmittedPower(
        double const I0,
        double const lambda,
        double const R,
        double const R0,
        double const r,
        double const w0
    );
    /**
     * @brief Legacy version of EnergyMaths::calcEmittedPower
     * @see EnergyMaths::calcEmittedPower
     */
    static double calcEmittedPowerLegacy(
        double const I0,
        double const lambda,
        double const R,
        double const R0,
        double const r,
        double const w0
    );

    /**
     * @brief Compute the emitted power for a subray such that the sum of the
     *  emitted energy by each subray matches the emitted energy when only a
     *  single ray is used.
     *
     * \f[
     *  P_e =  \frac{I_0}{n_{sr}} \Biggl(
     *      \exp\biggl[
     *          - \frac{2}{w^2}
     *              \left(\frac{\varphi_*}{\mathrm{BSQ}}i\right)^2
     *      \biggr]
     *      - \exp\biggl[
     *          - \frac{2}{w^2}
     *              \left(\frac{\varphi_*}{\mathrm{BSQ}}(i+1)\right)^2
     *      \biggr]
     *  \Biggr)
     * \f]
     *
     * Where
     *
     * \f[
     *  w = \varphi_* R
     * \f]
     *
     * Note that the previous expression can be simplified to reduce the
     * computational burden such that:
     *
     * \f[
     *  P_e = \frac{I_0}{n_{sr}} \Biggl(
     *      \exp\biggl[
     *          - \frac{2i^2}{\mathrm{BSQ}^2 R^2}
     *      \biggr]
     *      - \exp\biggl[
     *          - \frac{2(i+1)^2}{\mathrm{BSQ}^2 R^2}
     *      \biggr]
     *  \Biggr)
     * \f]
     *
     * @param I0 The average power of the device \f$I_0\f$.
     * @param R The target range (in meters) \f$R\f$.
     * @param beamDiv_mrad The beam divergence of the device (in milliradians)
     *  \f$\varphi_*\f$
     * @param w0 The beam waist radius \f$w_0\f$
     * @param BSQ The beam sample quality parameter governing the
     *  discrete approximation of the elliptical footprint \f$\mathrm{BSQ}\f$.
     * @param circleIter The iteration along the circle \f$i\f$.
     * @param numSubrays The number of subrays in the elliptical footprint
     *  approximation \f$n_{sr}\f$.
     * @return The emitted power for the corresponding subray.
     */
    static double calcSubrayWiseEmittedPower(
        double const I0,
        double const R,
        double const BSQ,
        double const circleIter,
        double const numSubrays
    );

    /**
     * @brief Solve the laser radar equation
	 *
     * <br/>
	 * Report title: Signature simulation and signal analysis for 3-D laser
	 * radar
	 * <br/>
	 * Report authors: Tomas Carlsson, Ove Steinvall and Dietmar Letalick
     *
     * \f[
     *  P_r = \frac{
     *          I_0 D_r^2 \eta_s \sigma
     *      }{
     *          4 \pi R^4 B_t^2
     *      }
     *      \exp\left[-\left(
     *          \frac{2\pi^2r^2w_0^2}{\lambda^2\left(R_0^2 + R^2\right)} +
     *          2Ra_e
     *      \right)\right]
     * \f]
     * @param I0 Average power
     * @param lambda Wavelength
     * @param R Target range
     * @param R0 Minimum range
     * @param r Radius
     * @param w0 beam waist radius
     * @param Dr2 Squared receiver diameter
     * @param Bt2 Squared beam divergence
     * @param etaSys Efficiency of scanning device
     * @param ae Atmospheric extinction coefficient
     * @param sigma Cross section between target area and incidence angle
     * @return Calculated received power
     */
    static double calcReceivedPower(
        double const I0,
        double const lambda,
        double const R,
        double const R0,
        double const r,
        double const w0,
        double const Dr2,
        double const Bt2,
        double const etaSys,
        double const ae,
        double const sigma
    );
    /**
     * @brief Legacy version of EnergyMaths::calcReceivedPower
     * @param Pe The emitted power
     * @param etaAtm The atmospheric factor
     * @see EnergyMaths::calcReceivedPower
     * @see EnergyMaths::calcAtmosphericFactor
     */
    static double calcReceivedPowerLegacy(
        double const Pe,
        double const Dr2,
        double const R,
        double const Bt2,
        double const etaSys,
        double const etaAtm,
        double const sigma
    );

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
	 *  C_{S} = 4{\pi} \cdot f \cdot A_{lf} \cdot \cos(\theta)
	 * \f]
	 *
	 * <br/>
	 * Paper DOI: 10.1016/j.isprsjprs.2010.06.007
	 *
	 * @return Cross section
	 */
    static double calcCrossSection(
        double const f,
        double const Alf,
        double const theta
    );


    // ***  LIGHTING  *** //
    // ****************** //
    static double computeBDRF(
        Material const &mat, double const incidenceAngle
    );
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
     *  \mathrm{BDRF}_{\mathrm{PHONG}} = \bigl(1-K_s\bigr) +
     *      K_s {\cos(2\varphi)}^{N_s}
     * \f]
     *
     * The final BDRF will often be the Phong BDRF multiplied by the
     *  reflectance \f$\rho\f$:
     *
     * \f[
     *  \mathrm{BDRF} = \rho \mathrm{BDRF}_{\mathrm{PHONG}} = \rho \biggl(
     *      \bigl(1-K_s\bigr) + K_s {\cos(2\varphi)}^{N_s} \biggr)
     * \f]
     *
	 */
    static double phongBDRF(
        double const incidenceAngle,
        double const targetSpecularity,
        double const targetSpecularExponent
    );
};