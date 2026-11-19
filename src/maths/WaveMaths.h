#pragma once

#include <maths/PulseShape.h>

/** A dense recording anchored at the earliest return plus the pulse lower
 * bound. */
struct WaveformGrid
{
  double firstSampleRange_m = 0;
  double spacing_m = 0;
  int sampleCount = 0;
  double rangeAt(int index) const
  {
    return firstSampleRange_m + index * spacing_m;
  }
};

class WaveMaths
{
public:
  static WaveformGrid recordingGrid(double minReturn_m,
                                    double maxReturn_m,
                                    PulseShape const& pulse,
                                    double spacing_m,
                                    double maxSpan_m);
};
