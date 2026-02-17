#include <helios/maths/EnergyMaths.h>
#include <helios/util/HeliosException.h>
#include <helios/util/logger/logging.hpp>

// ***  EMITTED / RECEIVED POWER  *** //
// ********************************** //
// Space distribution equation to calculate the beam energy decreasing the
// further away from the center (Carlsson et al., 2001)
double
EnergyMaths::calcEmittedPower(double const I0,
                              double const lambda,
                              double const R,
                              double const R0,
                              double const r,
                              double const w0)
{
  double const denom = lambda * lambda * (R0 * R0 + R * R);
  return I0 / exp(PI_SQUARED_2 * r * r * w0 * w0 / denom);
}

double
EnergyMaths::calcEmittedPowerLegacy(double const I0,
                                    double const lambda,
                                    double const R,
                                    double const R0,
                                    double const r,
                                    double const w0)
{
  double const denom = M_PI * w0 * w0;
  double const omega = (lambda * R) / denom;
  double const omega0 = (lambda * R0) / denom;
  double const w = w0 * sqrt(omega0 * omega0 + omega * omega);
  return I0 * exp((-2 * r * r) / (w * w));
}

double
EnergyMaths::calcSubrayWiseEmittedPower(double const reversedI0,
                                        double const w0,
                                        double const w,
                                        double const radius,
                                        double const prevRadius,
                                        double const numSubrays)
{
  return EnergyMaths::calcSubrayWiseEmittedPowerFast(
    M_PI * reversedI0 * (w0 * w0) / (2 * numSubrays),
    w * w,
    -2.0 * radius * radius,
    -2.0 * prevRadius * prevRadius);
}

double
EnergyMaths::calcSubrayWiseEmittedPowerFast(
  double const deviceConstantExpression,
  double const wSquared,
  double const negRadiusSquaredx2,
  double const negPrevRadiusSquaredx2)
{
  return deviceConstantExpression *
         (std::exp(negPrevRadiusSquaredx2 / wSquared) - // inner radius
          std::exp(negRadiusSquaredx2 / wSquared)       // outer radius
         );
}

// Laser radar equation "Signature simulation..." (Carlsson et al., 2000)
double
EnergyMaths::calcReceivedPower(double const I0,
                               double const lambda,
                               double const R,
                               double const R0,
                               double const r,
                               double const w0,
                               double const Dr2,
                               double const Bt2,
                               double const etaSys,
                               double const ae,
                               double const sigma)
{
  return calcReceivedPowerFast(I0,
                               lambda * lambda,
                               R,
                               R * R,
                               R0 * R0,
                               r * r,
                               w0 * w0,
                               Dr2,
                               Bt2,
                               etaSys,
                               ae,
                               sigma);
}

double
EnergyMaths::calcReceivedPowerFast(double const I0,
                                   double const lambdaSquared,
                                   double const R,
                                   double const RSquared,
                                   double const R0Squared,
                                   double const rSquared,
                                   double const w0Squared,
                                   double const Dr2,
                                   double const Bt2,
                                   double const etaSys,
                                   double const ae,
                                   double const sigma)
{
  double const numer = I0 * Dr2 * etaSys * sigma;
  double const expon = exp((PI_SQUARED_2 * rSquared * w0Squared) /
                             (lambdaSquared * (R0Squared + RSquared)) +
                           2 * R * ae);
  double const denom = PI_4 * RSquared * RSquared * Bt2 * expon;
  return numer / denom;
}

double
EnergyMaths::calcReceivedPowerLegacy(double const Pe,
                                     double const Dr2,
                                     double const R,
                                     double const Bt2,
                                     double const etaSys,
                                     double const etaAtm,
                                     double const sigma)
{
  return (Pe * Dr2) / (PI_4 * pow(R, 4) * Bt2) * etaSys * etaAtm * sigma;
}

double
EnergyMaths::calcReceivedPowerImproved(double const Pe,
                                       double const Dr2,
                                       double const R,
                                       double const targetArea,
                                       double const etaSys,
                                       double const etaAtm,
                                       double const sigma)
{
  double const denom = 16.0 * targetArea * R * R;
  return EnergyMaths::calcReceivedPowerImprovedFast(
    Pe, Dr2, denom, etaSys, etaAtm, sigma);
}

double
EnergyMaths::calcReceivedPowerImprovedFast(double const Pe,
                                           double const Dr2,
                                           double const denom,
                                           double const etaSys,
                                           double const etaAtm,
                                           double const sigma)
{
  return Pe * Dr2 * etaSys * etaAtm * sigma / denom;
}

// ***  ATMOSPHERIC STUFF  *** //
// *************************** //
// Energy left after attenuation by air particles in range [0,1]
double
EnergyMaths::calcAtmosphericFactor(double const R, double const ae)
{
  return exp(-2 * R * ae);
}

// ***  CROSS-SECTION  *** //
// *********************** //
// ALS Simplification "Radiometric Calibration..." (Wagner, 2010) Eq. 14
double
EnergyMaths::calcCrossSection(double const f, double const Alf)
{
  return PI_4 * f * Alf;
}

// ***  LIGHTING  *** //
// ****************** //
double
EnergyMaths::computeBDRF(Material const& mat, double const incidenceAngle)
{
  // Supported lighting models
  if (mat.isPhong()) {
    return mat.reflectance *
           EnergyMaths::phongBDRF(
             incidenceAngle, mat.specularity, mat.specularExponent) *
           std::cos(incidenceAngle);
  } else if (mat.isLambert()) {
    return mat.reflectance * std::cos(incidenceAngle);
  } else if (mat.isDirectionIndependent()) {
    return mat.reflectance;
  }
  // Not acceptable lighting model
  std::stringstream ss;
  ss << "Unexpected lighting model for material \"" << mat.name << "\"";
  logging::ERR(ss.str());
  throw HeliosException("Unexpected lighting model.");
}

// Phong reflection model "Normalization of Lidar Intensity..." (Jutzi and
// Gross, 2009)
double
EnergyMaths::phongBDRF(double const incidenceAngle,
                       double const targetSpecularity,
                       double const targetSpecularExponent)
{
  return EnergyMaths::phongBDRFFast(incidenceAngle,
                                    std::cos(incidenceAngle),
                                    targetSpecularity,
                                    targetSpecularExponent);
}

double
EnergyMaths::phongBDRFFast(double const incidenceAngle,
                           double const cosIncidenceAngle,
                           double const targetSpecularity,
                           double const targetSpecularExponent)
{
  double const ks = targetSpecularity;
  double const kd = (1 - ks);
  double const specularAngle = 2 * incidenceAngle;
  double const specular =
    ks * pow(std::abs(cos(specularAngle)), targetSpecularExponent) /
    cosIncidenceAngle;
  return kd + specular;
}
