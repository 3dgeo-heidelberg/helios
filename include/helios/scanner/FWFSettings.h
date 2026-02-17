#pragma once

#include <helios/assetloading/Asset.h>

#include <ostream>
#include <sstream>
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
   * @brief Bin size for discretization (nanoseconds)
   */
  double binSize_ns = 0.25;
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
   * @brief Beam divergence (radians)
   */
  double beamDivergence_rad = 0.0003;
  /**
   * @brief Pulse length (nanoseconds)
   */
  double pulseLength_ns = 4.0;
  /**
   * @brief Beam sample quality
   */
  int beamSampleQuality = 3;
  /**
   * @brief Window size to iterate over discretization (nanoseconds)
   */
  double winSize_ns = pulseLength_ns / 4.0;
  /**
   * @brief Max full wave range (nanoseconds)
   */
  double maxFullwaveRange_ns = 0.0;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for full waveform settings
   */
  FWFSettings() {}

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
       << "binSize_ns = " << binSize_ns << "\n"
       << "minEchoWidth = " << minEchoWidth << "\n"
       << "peakEnergy = " << peakEnergy << "\n"
       << "apertureDiameter = " << apertureDiameter << "\n"
       << "scannerEfficiency = " << scannerEfficiency << "\n"
       << "atmosphericVisibility = " << atmosphericVisibility << "\n"
       << "scannerWaveLength = " << scannerWaveLength << "\n"
       << "beamDivergence_rad = " << beamDivergence_rad << "\n"
       << "pulseLength_ns = " << pulseLength_ns << "\n"
       << "beamSampleQuality = " << beamSampleQuality << "\n"
       << "winSize_ns = " << winSize_ns << "\n"
       << "maxFullwaveRange_ns = " << maxFullwaveRange_ns << "\n";
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
