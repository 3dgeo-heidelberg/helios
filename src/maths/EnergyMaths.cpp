#include <EnergyMaths.h>
#include <util/HeliosException.h>
#include <util/logger/logging.hpp>

// ***  EMITTED / RECEIVED POWER  *** //
// ********************************** //
double
EnergyMaths::calcSubrayWiseEmittedPower(double const reversedI0,
                                        double const w0,
                                        double const w,
                                        double const radius,
                                        double const prevRadius,
                                        double const numSubrays)
{
  return EnergyMaths::calcSubrayWiseEmittedPowerFast(
    PI * reversedI0 * (w0 * w0) / (2 * numSubrays),
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

double
EnergyMaths::calcReceivedPowerFast(double const Pe,
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
EnergyMaths::computeBRDF(Material const& mat, double const incidenceAngle)
{
  // Supported lighting models
  if (mat.isPhong()) {
    return mat.reflectance *
           EnergyMaths::phongBRDF(
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
EnergyMaths::phongBRDF(double const incidenceAngle,
                       double const targetSpecularity,
                       double const targetSpecularExponent)
{
  return EnergyMaths::phongBRDFFast(incidenceAngle,
                                    std::cos(incidenceAngle),
                                    targetSpecularity,
                                    targetSpecularExponent);
}

double
EnergyMaths::phongBRDFFast(double const incidenceAngle,
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

double
EnergyMaths::computeBRDFFromCosine(Material const& mat, double incidenceCosine)
{
  if (mat.isPhong()) {
    double const cosDoubleAngle = 2.0 * incidenceCosine * incidenceCosine - 1.0;
    return mat.reflectance *
           ((1.0 - mat.specularity) * incidenceCosine +
            mat.specularity *
              std::pow(std::abs(cosDoubleAngle), mat.specularExponent));
  } else if (mat.isLambert()) {
    return mat.reflectance * incidenceCosine;
  } else if (mat.isDirectionIndependent()) {
    return mat.reflectance;
  }
  logging::ERR("Unexpected lighting model for material \"" + mat.name + "\"");
  throw HeliosException("Unexpected lighting model.");
}
