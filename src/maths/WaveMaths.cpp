#include <WaveMaths.h>

// ***   T I M E   *** //
// ******************* //
int
WaveMaths::calcPropagationTimeLegacy(std::vector<double>& timeWave,
                                     int const numBins,
                                     double const binSize,
                                     double const pulseLength,
                                     double const pulseLengthDivisor)
{
  // Prepare variables
  double const tau = pulseLength / pulseLengthDivisor;
  double t, t_tau, pt;
  double peakValue = 0;
  int peakIndex = 0;
  // Do forward iterative process to compute nodes and pick max
  for (int i = 0; i < numBins; ++i) {
    t = i * binSize;
    t_tau = t / tau;
    pt = (t_tau * t_tau) * std::exp(-t_tau);
    timeWave[i] = pt;
    if (pt > peakValue) {
      peakValue = pt;
      peakIndex = i;
    }
  }
  // Return index of peak value
  return peakIndex;
}
