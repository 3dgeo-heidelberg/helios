#include <maths/MathConstants.h>
#include <maths/PulseShape.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
double
validatedFwhm(double fwhm_ns, double tolerance)
{
  double const fwhm_m = fwhm_ns * (SPEEDofLIGHT_mPerNanosec / 2.0);
  if (!std::isfinite(fwhm_ns) || fwhm_ns <= 0 || !std::isfinite(fwhm_m) ||
      fwhm_m <= 0)
    throw std::invalid_argument(
      "Pulse FWHM must be finite, positive and representable in metres");
  if (!std::isfinite(tolerance) || tolerance <= 0 || tolerance >= 1)
    throw std::invalid_argument(
      "recordingTailTolerance must be between 0 and 1");
  return fwhm_m;
}
}

std::shared_ptr<PulseShape>
PulseShape::create(std::string const& shape,
                   double fwhm_ns,
                   double tolerance,
                   double mean_ns)
{
  if (shape == "gamma")
    return std::make_shared<GammaPulse>(fwhm_ns, tolerance);
  if (shape == "gaussian")
    return std::make_shared<GaussianPulse>(fwhm_ns, tolerance, mean_ns);
  throw std::invalid_argument("pulseShape must be gamma or gaussian");
}

GaussianPulse::GaussianPulse(double fwhm_ns, double tolerance, double mean_ns)
  : mean_m(mean_ns * (SPEEDofLIGHT_mPerNanosec / 2.0))
  , sigma_m(validatedFwhm(fwhm_ns, tolerance) /
            (2 * std::sqrt(2 * std::log(2.0))))
  , extent_m(sigma_m * std::sqrt(-2 * std::log(tolerance)))
{
  if (sigma_m <= 0 || !std::isfinite(mean_m) ||
      !std::isfinite(mean_m - extent_m) || !std::isfinite(mean_m + extent_m))
    throw std::invalid_argument(
      "Gaussian pulse bounds cannot be represented in metres");
}

double
GaussianPulse::evaluate(double offset_m) const
{
  double const z = (offset_m - mean_m) / sigma_m;
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
  : rho_m(validatedFwhm(fwhm_ns, tolerance) / 3.394680670846504)
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

SampledPulse::SampledPulse(std::vector<double> offsets_ns,
                           std::vector<double> amplitudes,
                           std::optional<double> peak_offset_ns)
  : originalOffsets_ns(offsets_ns)
  , values(std::move(amplitudes))
  , explicitPeak_ns(peak_offset_ns)
{
  if (offsets_ns.size() < 2 || offsets_ns.size() != values.size())
    throw std::invalid_argument(
      "Sampled pulse requires equal arrays with at least two samples");
  offsets_m.reserve(offsets_ns.size());
  for (size_t i = 0; i < offsets_ns.size(); ++i) {
    double const offset = offsets_ns[i] * (SPEEDofLIGHT_mPerNanosec / 2.0);
    if (!std::isfinite(offsets_ns[i]) || !std::isfinite(offset) ||
        (i && (offset <= offsets_m.back() ||
               !std::isfinite(offset - offsets_m.back()))))
      throw std::invalid_argument(
        "Pulse offsets must be finite, strictly increasing and representable");
    if (!std::isfinite(values[i]) || values[i] < 0)
      throw std::invalid_argument(
        "Pulse amplitudes must be finite and nonnegative");
    offsets_m.push_back(offset);
  }
  double const maximum = *std::max_element(values.begin(), values.end());
  if (maximum <= 0)
    throw std::invalid_argument("Sampled pulse must have a positive maximum");
  if (!peak_offset_ns && std::count(values.begin(), values.end(), maximum) != 1)
    throw std::invalid_argument(
      "Ambiguous pulse maximum requires an explicit peak offset");
  auto const peakIndex =
    std::max_element(values.begin(), values.end()) - values.begin();
  for (double& value : values)
    value /= maximum;
  peak_m = peak_offset_ns ? *peak_offset_ns * (SPEEDofLIGHT_mPerNanosec / 2.0)
                          : offsets_m[peakIndex];
  if (!std::isfinite(peak_m) || evaluate(peak_m) != 1.0)
    throw std::invalid_argument(
      "Explicit peak offset must lie at a pulse maximum");
}

double
SampledPulse::evaluate(double offset_m) const
{
  if (!std::isfinite(offset_m) || offset_m < offsets_m.front() ||
      offset_m > offsets_m.back())
    return 0;
  auto const upper =
    std::upper_bound(offsets_m.begin(), offsets_m.end(), offset_m);
  if (upper == offsets_m.end())
    return values.back();
  size_t const i = static_cast<size_t>(upper - offsets_m.begin()) - 1;
  double const fraction =
    (offset_m - offsets_m[i]) / (offsets_m[i + 1] - offsets_m[i]);
  return (1 - fraction) * values[i] + fraction * values[i + 1];
}

std::vector<double>
SampledPulse::offsetsNanoseconds() const
{
  return originalOffsets_ns;
}
