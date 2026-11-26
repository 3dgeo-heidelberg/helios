#pragma once

#include "Asset.h"
#include "BeamSampling.h"
#include <maths/MathConstants.h>
#include <maths/PulseShape.h>

#include <cmath>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>

/**
 * @brief Full Waveform settings
 */
class FWFSettings : public Asset
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Temporal sample spacing (seconds)
   */
  double sampleSpacing_s = 0.25e-9;
  std::string pulseShape = "gamma";
  std::shared_ptr<SampledPulse> sampledPulse;
  double recordingTailTolerance = 1e-4;
  double evaluationTailTolerance = 1e-4;
  /**
   * @brief Minimum echo width
   */
  double minEchoWidth = 2.5;
  /**
   * @brief Peak energy
   */
  double peakEnergy = 500.0;
  /**
   * @brief Aperture diameter
   */
  double apertureDiameter = 0.15;
  /**
   * @brief Scanner efficiency
   */
  double scannerEfficiency = 0.9;
  /**
   * @brief Atmospheric visibility
   */
  double atmosphericVisibility = 0.9;
  /**
   * @brief Scanner wave length
   */
  double scannerWaveLength = 1550.0;
  /**
   * @brief Pulse length (seconds)
   */
  double pulseLength_s = 4.0e-9;
  /** Sampling geometry; polar preserves the default ray pattern. */
  BeamSamplingMethod beamSamplingMethod = BeamSamplingMethod::Polar;
  /** Ring count, or Cartesian axis cell count (2 * quality - 1). */
  int beamSampleQuality = 3;
  /** Normalized radial cutoff (polar) or per-axis cutoff (Cartesian). */
  double beamSamplingFactor = 2.0;
  /**
   * @brief Peak validation window (seconds)
   */
  double peakWindow_s = 1e-9;
  /**
   * @brief Maximum recording span (seconds), zero means unlimited
   */
  double maxRecordingSpan_s = 0.0;

  double sampleSpacingMeters() const
  {
    return sampleSpacing_s * (SPEEDofLIGHT_mPerSec / 2.0);
  }
  double peakWindowMeters() const
  {
    return peakWindow_s * (SPEEDofLIGHT_mPerSec / 2.0);
  }
  double maxRecordingSpanMeters() const
  {
    return maxRecordingSpan_s * (SPEEDofLIGHT_mPerSec / 2.0);
  }

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for full waveform settings
   */
  FWFSettings() {}

  /**
   * @brief Validate the sampleSpacing_s and peakWindow_s attributes to ensure
   * they are valid
   * @throws std::invalid_argument if any setting is invalid
   */
  void validate() const
  {
    validateBeamSampling();
    if (!std::isfinite(evaluationTailTolerance) ||
        evaluationTailTolerance < 0 || evaluationTailTolerance >= 1)
      throw std::invalid_argument("evaluationTailTolerance must be in [0, 1)");
    if (sampleSpacingMeters() <= 0 || !std::isfinite(sampleSpacingMeters()) ||
        !std::isfinite(peakWindowMeters()) ||
        !std::isfinite(maxRecordingSpanMeters()))
      throw std::invalid_argument(
        "Waveform durations must be representable as ranges");
    if (!std::isfinite(sampleSpacing_s) || sampleSpacing_s <= 0)
      throw std::invalid_argument("sampleSpacing_s must be finite and > 0");
    if (!std::isfinite(peakWindow_s) || peakWindow_s < 2 * sampleSpacing_s ||
        peakWindow_s / sampleSpacing_s > 1000000000)
      throw std::invalid_argument(
        "peakWindow_s must be >= 2 * sampleSpacing_s and representable");
    if (!std::isfinite(maxRecordingSpan_s) || maxRecordingSpan_s < 0)
      throw std::invalid_argument("maxRecordingSpan_s must be finite and >= 0");
    if (!std::isfinite(recordingTailTolerance) || recordingTailTolerance <= 0 ||
        recordingTailTolerance >= 1)
      throw std::invalid_argument(
        "recordingTailTolerance must be between 0 and 1");
    if (pulseShape != "gamma" && pulseShape != "gaussian" &&
        pulseShape != "sampled")
      throw std::invalid_argument(
        "pulseShape must be gamma, gaussian or sampled");
    if (pulseShape == "sampled" && !sampledPulse)
      throw std::invalid_argument("sampled pulse shape requires sampledPulse");
  }
  void validateBeamSampling() const
  {
    BeamSamplingSettings{ beamSamplingMethod,
                          beamSampleQuality,
                          beamSamplingFactor }
      .validate();
  }

  // ***  TO STRING  *** //
  // ******************* //
  /**
   * @brief Obtain the string representation of the scanner settings
   * @return String representing the scanner settings
   */
  virtual inline std::string toString() const
  {
    std::stringstream ss;
    ss << "FWFSettings \"" << id << "\":\n"
       << "evaluationTailTolerance = " << evaluationTailTolerance << "\n"
       << "sampleSpacing_s = " << sampleSpacing_s << "\n"
       << "pulseShape = " << pulseShape << "\n"
       << "sampledPulse = " << (sampledPulse ? "present" : "none") << "\n"
       << "recordingTailTolerance = " << recordingTailTolerance << "\n"
       << "minEchoWidth = " << minEchoWidth << "\n"
       << "peakEnergy = " << peakEnergy << "\n"
       << "apertureDiameter = " << apertureDiameter << "\n"
       << "scannerEfficiency = " << scannerEfficiency << "\n"
       << "atmosphericVisibility = " << atmosphericVisibility << "\n"
       << "scannerWaveLength = " << scannerWaveLength << "\n"
       << "pulseLength_s = " << pulseLength_s << "\n"
       << "beamSamplingMethod = " << ::toString(beamSamplingMethod) << "\n"
       << "beamSampleQuality = " << beamSampleQuality << "\n"
       << "beamSamplingFactor = " << beamSamplingFactor << "\n"
       << "peakWindow_s = " << peakWindow_s << "\n"
       << "maxRecordingSpan_s = " << maxRecordingSpan_s << "\n";
    return ss.str();
  }
  /**
   * @brief Overload of << operator for output streams
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  const FWFSettings& settings)
  {
    out << settings.toString();
    return out;
  }
};
