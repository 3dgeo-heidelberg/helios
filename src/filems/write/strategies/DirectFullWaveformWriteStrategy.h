#pragma once

#include <filems/write/strategies/WriteStrategy.h>
#include <scanner/detector/FullWaveform.h>

#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <vector>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to directly write full
 *  waveform data to a file.
 * @see filems::WriteStrategy
 * @see filems::SimpleSyncFileFullWaveformWriter
 */
class DirectFullWaveformWriteStrategy
  : public WriteStrategy<FullWaveform const&>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The output file stream to do the writing
   */
  std::ofstream& ofs;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for direct full waveform write strategy
   * @see DirectFullWaveformWriteStrategy::ofs
   */
  DirectFullWaveformWriteStrategy(std::ofstream& ofs)
    : ofs(ofs)
  {
  }
  virtual ~DirectFullWaveformWriteStrategy() = default;

  // ***  WRITE STRATEGY INTERFACE  *** //
  // ********************************** //
  /**
   * @brief Write full waveform data to file
   * @param fullwaveForm The full waveform to be written
   * @see SyncFileWriter::_write
   */
  void write(FullWaveform const& fullWaveform) override
  {
    ofs << fullWaveformToString(fullWaveform);
  }

protected:
  // ***  UTILS  *** //
  // *************** //
  /**
   * @brief Build a string from fullwave data
   * @param fw The full waveform data itself
   * @return String with full waveform data
   */
  virtual std::string fullWaveformToString(FullWaveform const& fw)
  {
    std::stringstream ss;
    ss << std::setprecision(4) << std::fixed << fw.fullwaveIndex << " "
       << fw.beamOrigin.x << " " << fw.beamOrigin.y << " " << fw.beamOrigin.z
       << " " << fw.beamDir.x << " " << fw.beamDir.y << " " << fw.beamDir.z
       << " " << fw.minTime << " " << fw.maxTime << " " << std::setprecision(9)
       << std::fixed << fw.gpsTime / 1000000000.0 << " ";

    std::copy(fw.fullwave.begin(),
              fw.fullwave.end() - 1,
              std::ostream_iterator<double>(ss, " "));
    ss << fw.fullwave.back();
    ss << "\n";
    return ss.str();
  }
};

}
}
