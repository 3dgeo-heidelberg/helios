#pragma once

#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/strategies/DirectFullWaveformWriteStrategy.h>

#include <memory>
#include <vector>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write full
 *  waveform data directly to a file.
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectFullWaveformWriteStrategy
 */
class SimpleSyncFileFullWaveformWriter
  : public SimpleSyncFileWriter<FullWaveform const&>
{

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous file full waveform writer constructor
   * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
   */
  explicit SimpleSyncFileFullWaveformWriter(
    const std::string& path,
    std::ios_base::openmode om = std::ios_base::app)
    : SimpleSyncFileWriter<FullWaveform const&>(path, om)
  {
    this->writeStrategy =
      std::make_shared<DirectFullWaveformWriteStrategy>(this->ofs);
  }
  virtual ~SimpleSyncFileFullWaveformWriter() = default;
};

}
}
