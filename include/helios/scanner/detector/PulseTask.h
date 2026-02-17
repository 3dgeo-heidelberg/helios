#pragma once

#include <helios/noise/NoiseSource.h>
#include <helios/noise/RandomnessGenerator.h>
#if DATA_ANALYTICS >= 2
#include <helios/dataanalytics/HDA_PulseRecorder.h>
using helios::analytics::HDA_PulseRecorder;
#endif

#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Pulse task interface
 */
class PulseTask
{
public:
  // ***  O P E R A T O R  *** //
  // ************************* //
  /**
   * @brief Pulse task void functor. It is necessary due to compatibility
   *  reasons.
   */
  virtual void operator()() = 0;
  /**
   * @brief Pulse task runnable functor
   * @param apMatrix Reference to matrix to be used to compute Marquardt
   * fitter
   * @param randGen A randomness generator
   * @param randGen2 Another randomness generator
   * @param intersectionHandlingNoiseSource Noise source to be used at
   * intersection handling if necessary
   */
  virtual void operator()(std::vector<std::vector<double>>& apMatrix,
                          RandomnessGenerator<double>& randGen,
                          RandomnessGenerator<double>& randGen2,
                          NoiseSource<double>& intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
                          ,
                          std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
                          ) = 0;
};
