#pragma once

#include <glm/glm.hpp>

#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a full waveform
 */
class FullWaveform
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The full wave vector containing the values of the full
   *  waveform itself
   */
  std::vector<double> fullwave;
  /**
   * @brief The index for the full wave
   */
  int fullwaveIndex;
  /**
   * @brief Minimum hit time (nanoseconds)
   */
  double minTime;
  /**
   * @brief Maximum hit time (nanoseconds)
   */
  double maxTime;
  /**
   * @brief The coordinates of the beam origin
   */
  glm::dvec3 beamOrigin;
  /**
   * @brief The director vector of the beam
   */
  glm::dvec3 beamDir;
  /**
   * @brief The GPS time associated to the full wave (nanoseconds)
   */
  double gpsTime;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default full waveform constructor
   */
  FullWaveform() = default;
  /**
   * @brief Built full waveform directly from given attributes
   */
  FullWaveform(std::vector<double> const& fullwave,
               int const fullwaveIndex,
               double const minTime,
               double const maxTime,
               glm::dvec3 const& beamOrigin,
               glm::dvec3 const& beamDir,
               double const gpsTime)
    : fullwave(fullwave)
    , fullwaveIndex(fullwaveIndex)
    , minTime(minTime)
    , maxTime(maxTime)
    , beamOrigin(beamOrigin)
    , beamDir(beamDir)
    , gpsTime(gpsTime)
  {
  }
  FullWaveform(FullWaveform const& fw)
  {
    fullwave = fw.fullwave;
    fullwaveIndex = fw.fullwaveIndex;
    minTime = fw.minTime;
    maxTime = fw.maxTime;
    beamOrigin = fw.beamOrigin;
    beamDir = fw.beamDir;
    gpsTime = fw.gpsTime;
  }
  virtual ~FullWaveform() = default;

  // ***  OPERATORS  *** //
  // ******************* //
  friend std::ostream& operator<<(std::ostream& out, FullWaveform& fw)
  {
    out << fw.fullwaveIndex << ", " << fw.minTime << ", " << fw.maxTime << ", "
        << fw.beamOrigin.x << ", " << fw.beamOrigin.y << ", " << fw.beamOrigin.z
        << ", " << fw.beamDir.x << ", " << fw.beamDir.y << ", " << fw.beamDir.z
        << ", " << fw.gpsTime;
    for (double const& fwi : fw.fullwave)
      out << ", " << fwi;
    return out;
  }
};
