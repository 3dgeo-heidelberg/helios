#pragma once

#include <memory>
#include <string>
#include <utility>

/** Immutable, unit-peak optical pulse evaluated in range-equivalent metres.
 * Recording bounds suggest an acquisition interval, never a pulse cutoff.
 */
class AnalyticPulse
{
public:
  virtual ~AnalyticPulse() = default;
  virtual double evaluate(double offset_m) const = 0;
  virtual double peakOffset() const = 0;
  virtual std::pair<double, double> recordingBounds() const = 0;

  static std::shared_ptr<AnalyticPulse> create(std::string const& shape,
                                               double fwhm_ns,
                                               double recordingTailTolerance);

protected:
  AnalyticPulse(double fwhm_ns, double recordingTailTolerance);
  double const fwhm_m;
};

/** Gaussian centred at +3 sigma, including its negative-offset tail. */
class GaussianPulse : public AnalyticPulse
{
public:
  explicit GaussianPulse(double fwhm_ns, double recordingTailTolerance = 1e-4);
  double evaluate(double offset_m) const override;
  double peakOffset() const override { return 3 * sigma_m; }
  std::pair<double, double> recordingBounds() const override
  {
    return { peakOffset() - extent_m, peakOffset() + extent_m };
  }

private:
  double const sigma_m;
  double const extent_m;
};

/** Causal gamma pulse with onset at zero and peak at 2*rho. */
class GammaPulse : public AnalyticPulse
{
public:
  explicit GammaPulse(double fwhm_ns, double recordingTailTolerance = 1e-4);
  double evaluate(double offset_m) const override;
  double peakOffset() const override { return 2 * rho_m; }
  std::pair<double, double> recordingBounds() const override
  {
    return { 0, upperBound_m };
  }

private:
  double const rho_m;
  double const upperBound_m;
};
