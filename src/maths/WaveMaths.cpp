#include <maths/MathConstants.h>
#include <maths/WaveMaths.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

WaveformGrid
WaveMaths::recordingGrid(double minReturn_m,
                         double maxReturn_m,
                         PulseShape const& pulse,
                         double spacing_m,
                         double maxSpan_m)
{
  if (!std::isfinite(minReturn_m) || !std::isfinite(maxReturn_m) ||
      minReturn_m > maxReturn_m || !std::isfinite(spacing_m) ||
      spacing_m <= 0 || !std::isfinite(maxSpan_m) || maxSpan_m < 0)
    throw std::invalid_argument("Invalid waveform recording interval");

  auto const bounds = pulse.recordingBounds();
  double const first = minReturn_m + bounds.first;
  double const upper = maxReturn_m + bounds.second;
  double last = std::ceil((upper - first) / spacing_m);

  if (maxSpan_m > 0)
    last = std::min(last, std::floor(maxSpan_m / spacing_m));

  double const count = last + 1;

  // Relative indices must fit downstream signed indexing (including
  // peak-window arithmetic), and sample coordinates must remain finite.
  if (!std::isfinite(first) || !std::isfinite(upper) || !std::isfinite(count) ||
      count < 1 || count > std::numeric_limits<int>::max() / 2 ||
      count > std::vector<double>().max_size() ||
      !std::isfinite(first + last * spacing_m))
    throw std::length_error("Waveform recording grid is too large");

  return { first, spacing_m, static_cast<int>(count) };
}
