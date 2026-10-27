#pragma once

#include <maths/PulseShape.h>

/** A dense recording on a range grid anchored at zero. */
struct WaveformGrid
{
  double firstSampleIndex = 0;
  double spacing_m = 0;
  int sampleCount = 0;
  double rangeAt(int index) const
  {
    return (firstSampleIndex + index) * spacing_m;
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
