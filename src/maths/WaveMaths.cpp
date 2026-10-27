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
  double const first = std::floor((minReturn_m + bounds.first) / spacing_m);
  double last = std::ceil((maxReturn_m + bounds.second) / spacing_m);
  if (maxSpan_m > 0)
    last = std::min(last, first + std::floor(maxSpan_m / spacing_m));
  double const count = last - first + 1;
  // Integer indices must remain exactly representable, and fit downstream
  // signed indexing (including peak-window arithmetic).
  if (!std::isfinite(first) || !std::isfinite(last) ||
      std::abs(first) > 0x1p52 || std::abs(last) > 0x1p52 ||
      !std::isfinite(count) || count < 1 ||
      count > std::numeric_limits<int>::max() / 2 ||
      count > std::vector<double>().max_size() ||
      !std::isfinite(first * spacing_m) || !std::isfinite(last * spacing_m))
    throw std::length_error("Waveform recording grid is too large");
  return { first, spacing_m, static_cast<int>(count) };
}
