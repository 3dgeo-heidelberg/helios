#pragma once

#include <filems/util/ZipRecordIO.h>
#include <filems/write/strategies/DirectPulseWriteStrategy.h>

namespace helios {
namespace filems {

class ZipPulseWriteStrategy : public DirectPulseWriteStrategy
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The zipping output stream to do the writing. It must be
   *  associated to the file output stream of the parent
   *  DirectPulseWriteStrategy.
   */
  boost::iostreams::filtering_ostream& oa;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for zip pulse write strategy
   * @see ZipPulseWriteStrategy::oa
   * @see DirectPulseWriteStrategy::DirectPulseWriteStrategy
   */
  ZipPulseWriteStrategy(std::ofstream& ofs,
                        boost::iostreams::filtering_ostream& oa)
    : DirectPulseWriteStrategy(ofs)
    , oa(oa)
  {
  }
  ~ZipPulseWriteStrategy() override = default;

  // ***  WRITE STRATEGY INTERFACE *** //
  // ********************************* //
  /**
   * @brief Write pulse to compressed file
   * @see DirectPulseWriteStrategy::write
   */
  void write(PulseRecord const& pulseRecord) override
  {
    writeZippedStringRecord(oa, pulseToString(pulseRecord));
  }
};

}
}
