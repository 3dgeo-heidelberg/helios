#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

/** Immutable, zero-centered unit-peak optical pulse evaluated in
 * range-equivalent metres. Analytic recording bounds suggest an acquisition
 * interval, not a cutoff. Sampled pulses have finite support centered on their
 * selected input peak.
 */
class PulseShape
{
public:
  virtual ~PulseShape() = default;
  virtual double evaluate(double offset_m) const = 0;
  virtual std::pair<double, double> recordingBounds() const = 0;

  static std::shared_ptr<PulseShape> create(std::string const& shape,
                                            double fwhm_ns,
                                            double recordingTailTolerance);
};

/** Gaussian centered at zero. */
class GaussianPulse : public PulseShape
{
public:
  explicit GaussianPulse(double fwhm_ns, double recordingTailTolerance = 1e-4);
  double evaluate(double offset_m) const override;
  std::pair<double, double> recordingBounds() const override
  {
    return { -extent_m, extent_m };
  }

private:
  double const sigma_m;
  double const extent_m;
};

/** Causal gamma pulse with onset at -2*rho and peak at zero. */
class GammaPulse : public PulseShape
{
public:
  explicit GammaPulse(double fwhm_ns, double recordingTailTolerance = 1e-4);
  double evaluate(double offset_m) const override;
  std::pair<double, double> recordingBounds() const override
  {
    return { -2 * rho_m, upperBound_m - 2 * rho_m };
  }

private:
  double const rho_m;
  double const upperBound_m;
};

/** Immutable, unit-peak piecewise-linear pulse; zero outside supplied samples.
 */
class SampledPulse : public PulseShape
{
public:
  SampledPulse(std::vector<double> offsets_ns,
               std::vector<double> amplitudes,
               std::optional<double> peak_offset_ns = std::nullopt);
  double evaluate(double offset_m) const override;
  std::pair<double, double> recordingBounds() const override
  {
    return { offsets_m.front(), offsets_m.back() };
  }
  std::vector<double> offsetsNanoseconds() const;
  std::vector<double> const& amplitudes() const { return values; }
  std::optional<double> explicitPeakNanoseconds() const
  {
    return explicitPeak_ns;
  }

private:
  std::vector<double> originalOffsets_ns;
  std::vector<double> offsets_m;
  std::vector<double> values;
  std::optional<double> explicitPeak_ns;
};
