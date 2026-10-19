#pragma once

#include "Asset.h"

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
   * @brief Range sample spacing (metres)
   */
  double sampleSpacing_m = 0.03747405725;
  std::string pulseShape = "gamma";
  bool correctPeakOffset = true;
  double recordingTailTolerance = 1e-4;
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
   * @brief Pulse length (nanoseconds)
   */
  double pulseLength_ns = 4.0;
  /**
   * @brief Beam sample quality
   */
  int beamSampleQuality = 3;
  /** Tangent-plane sampling radius in units of the Gaussian 1/e^2 radius. */
  double beamSamplingFactor = 2.0;
  /**
   * @brief Peak validation window (metres)
   */
  double peakWindow_m = 0.149896229;
  /**
   * @brief Maximum recording span (metres), zero means unlimited
   */
  double maxRecordingSpan_m = 0.0;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for full waveform settings
   */
  FWFSettings() {}

  /**
   * @brief Validate the sampleSpacing_m and peakWindow_m attributes to ensure
   * they are valid
   * @throws std::invalid_argument if any setting is invalid
   */
  void validate() const
  {
    validateBeamSampling();
    if (!std::isfinite(sampleSpacing_m) || sampleSpacing_m <= 0)
      throw std::invalid_argument("sampleSpacing_m must be finite and > 0");
    if (!std::isfinite(peakWindow_m) || peakWindow_m < 2 * sampleSpacing_m ||
        peakWindow_m / sampleSpacing_m > 1000000000)
      throw std::invalid_argument(
        "peakWindow_m must be >= 2 * sampleSpacing_m and representable");
    if (!std::isfinite(maxRecordingSpan_m) || maxRecordingSpan_m < 0)
      throw std::invalid_argument("maxRecordingSpan_m must be finite and >= 0");
    if (!std::isfinite(recordingTailTolerance) || recordingTailTolerance <= 0 ||
        recordingTailTolerance >= 1)
      throw std::invalid_argument(
        "recordingTailTolerance must be between 0 and 1");
    if (pulseShape != "gamma" && pulseShape != "gaussian")
      throw std::invalid_argument("pulseShape must be gamma or gaussian");
  }
  void validateBeamSampling() const
  {
    if (beamSampleQuality < 1)
      throw std::invalid_argument("beamSampleQuality must be >= 1");
    if (!std::isfinite(beamSamplingFactor) || beamSamplingFactor <= 0.0)
      throw std::invalid_argument("beamSamplingFactor must be finite and > 0");
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
       << "sampleSpacing_m = " << sampleSpacing_m << "\n"
       << "pulseShape = " << pulseShape << "\n"
       << "correctPeakOffset = " << correctPeakOffset << "\n"
       << "recordingTailTolerance = " << recordingTailTolerance << "\n"
       << "minEchoWidth = " << minEchoWidth << "\n"
       << "peakEnergy = " << peakEnergy << "\n"
       << "apertureDiameter = " << apertureDiameter << "\n"
       << "scannerEfficiency = " << scannerEfficiency << "\n"
       << "atmosphericVisibility = " << atmosphericVisibility << "\n"
       << "scannerWaveLength = " << scannerWaveLength << "\n"
       << "pulseLength_ns = " << pulseLength_ns << "\n"
       << "beamSampleQuality = " << beamSampleQuality << "\n"
       << "beamSamplingFactor = " << beamSamplingFactor << "\n"
       << "peakWindow_m = " << peakWindow_m << "\n"
       << "maxRecordingSpan_m = " << maxRecordingSpan_m << "\n";
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
