#include <maths/AnalyticPulse.h>
#include <maths/MathConstants.h>

#include <cmath>
#include <stdexcept>

AnalyticPulse::AnalyticPulse(double fwhm_ns, double tolerance)
  : fwhm_m(fwhm_ns * (SPEEDofLIGHT_mPerNanosec / 2.0))
{
  if (!std::isfinite(fwhm_ns) || fwhm_ns <= 0 || !std::isfinite(fwhm_m) ||
      fwhm_m <= 0)
    throw std::invalid_argument(
      "Pulse FWHM must be finite, positive and representable in metres");
  if (!std::isfinite(tolerance) || tolerance <= 0 || tolerance >= 1)
    throw std::invalid_argument(
      "recordingTailTolerance must be between 0 and 1");
}

std::shared_ptr<AnalyticPulse>
AnalyticPulse::create(std::string const& shape,
                      double fwhm_ns,
                      double tolerance)
{
  if (shape == "gamma")
    return std::make_shared<GammaPulse>(fwhm_ns, tolerance);
  if (shape == "gaussian")
    return std::make_shared<GaussianPulse>(fwhm_ns, tolerance);
  throw std::invalid_argument("pulseShape must be gamma or gaussian");
}

GaussianPulse::GaussianPulse(double fwhm_ns, double tolerance)
  : AnalyticPulse(fwhm_ns, tolerance)
  , sigma_m(fwhm_m / (2 * std::sqrt(2 * std::log(2.0))))
  , extent_m(sigma_m * std::sqrt(-2 * std::log(tolerance)))
{
  if (sigma_m <= 0 || !std::isfinite(peakOffset() + extent_m))
    throw std::invalid_argument(
      "Gaussian pulse bounds cannot be represented in metres");
}

double
GaussianPulse::evaluate(double offset_m) const
{
  double const z = offset_m / sigma_m - 3;
  return std::exp(-0.5 * z * z);
}

namespace {
double
fallingFlankCrossing(double tolerance)
{
  auto logAmplitude = [](double x) {
    return 2.0 - std::log(4.0) + 2 * std::log(x) - x;
  };
  double lower = 2, upper = 4;
  double const logTolerance = std::log(tolerance);
  while (logAmplitude(upper) > logTolerance)
    upper *= 2;
  for (int i = 0; i < 80; ++i) {
    double const mid = (lower + upper) / 2;
    if (logAmplitude(mid) > logTolerance)
      lower = mid;
    else
      upper = mid;
  }
  return upper;
}
}

GammaPulse::GammaPulse(double fwhm_ns, double tolerance)
  : AnalyticPulse(fwhm_ns, tolerance)
  , rho_m(fwhm_m / 3.394680670846504)
  , upperBound_m(rho_m * fallingFlankCrossing(tolerance))
{
  if (rho_m <= 0 || !std::isfinite(upperBound_m))
    throw std::invalid_argument(
      "Gamma pulse bounds cannot be represented in metres");
}

double
GammaPulse::evaluate(double offset_m) const
{
  double const x = offset_m / rho_m;
  if (x <= 0 || !std::isfinite(x))
    return 0;
  // e^2/4 * x^2 * exp(-x), without overflowing x^2 for distant returns.
  // There is no tail cutoff.
  double const rootAmplitude = (x / 2) * std::exp(1 - x / 2);
  return rootAmplitude * rootAmplitude;
}
