#pragma once

#include <filems/util/ZipRecordIO.h>
#include <filems/write/strategies/DirectFullWaveformWriteStrategy.h>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Like DirectFullWaveformWriteStrategy but zipping the output
 * @see filems::DirectFullWaveformWriteStrategy
 */
class ZipFullWaveformWriteStrategy : public DirectFullWaveformWriteStrategy
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The zipping output stream to do the writing. It must be
   *  associated to the file output stream of the parent
   *  DirectFullWaveformWriteStrategy
   */
  boost::iostreams::filtering_ostream& oa;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for zip full waveform write strategy
   * @see ZipFullWaveformWriteStrategy::oa
   * @see DirectFullWaveformWriteStrategy::DirectFullWaveformWriteStrategy
   */
  ZipFullWaveformWriteStrategy(std::ofstream& ofs,
                               boost::iostreams::filtering_ostream& oa)
    : DirectFullWaveformWriteStrategy(ofs)
    , oa(oa)
  {
  }
  ~ZipFullWaveformWriteStrategy() override = default;

  // ***  WRITE STRATEGY INTERFACE *** //
  // ********************************* //
  /**
   * @brief Write full waveform to compressed file
   * @see DirectFullWaveformWriteStrategy::write
   */
  void write(FullWaveform const& fullWaveform) override
  {
    writeZippedStringRecord(oa, fullWaveformToString(fullWaveform));
  }
};

}
}
