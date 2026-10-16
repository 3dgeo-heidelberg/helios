#include <WaveMaths.h>

namespace {
// FWHM of x^2 exp(-x), expressed in units of tau.
constexpr double pulseFwhmFactor = 3.394680670846504;
}

double
WaveMaths::calcPulseDuration(double pulseLength)
{
  // Keep the historical sampling cutoff at 7*tau, beyond the falling half
  // maximum.
  return 7.0 * pulseLength / pulseFwhmFactor;
}

// ***   T I M E   *** //
// ******************* //
int
WaveMaths::calcPropagationTimeLegacy(std::vector<double>& timeWave,
                                     int const numBins,
                                     double const binSize,
                                     double const pulseLength)
{
  // Prepare variables
  double const tau = pulseLength / pulseFwhmFactor;
  double t, t_tau, pt;
  double peakValue = 0;
  int peakIndex = 0;
  // Do forward iterative process to compute nodes and pick max
  for (int i = 0; i < numBins; ++i) {
    t = i * binSize;
    t_tau = t / tau;
    // Analytic unit peak at t = 2*tau; independent of bin resolution.
    pt = (std::exp(2.0) / 4.0) * (t_tau * t_tau) * std::exp(-t_tau);
    timeWave[i] = pt;
    if (pt > peakValue) {
      peakValue = pt;
      peakIndex = i;
    }
  }
  // Return index of peak value
  return peakIndex;
}
