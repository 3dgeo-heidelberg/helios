#pragma once

#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/strategies/DirectFullWaveformWriteStrategy.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>
#include <scanner/detector/FullWaveform.h>

#include <memory>
#include <vector>

namespace helios {
namespace filems {

using std::make_shared;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write a vector
 *  of full waveforms directly to a file
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectFullWaveformWriteStrategy
 * @see FullWaveform
 * @see filems::SimpleSyncFileFullWaveformWriter
 */
class SimpleVectorialSyncFileFullWaveformWriter
  : public SimpleSyncFileWriter<vector<FullWaveform> const&>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The full waveform write strategy that is wrapped by the main
   *  write strategy in a vectorial fashion
   *  ( filems::SimpleSyncFileWriter::writeStrategy )
   * @see filems::DirectFullWaveformWriteStrategy
   */
  DirectFullWaveformWriteStrategy dfwws;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous file full waveform vector writer constructor
   * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
   */
  explicit SimpleVectorialSyncFileFullWaveformWriter(
    const std::string& path,
    std::ios_base::openmode om = std::ios_base::app)
    : SimpleSyncFileWriter<vector<FullWaveform> const&>(path, om)
    , dfwws(this->ofs)
  {
    this->writeStrategy =
      make_shared<VectorialWriteStrategy<FullWaveform>>(dfwws);
  }
  virtual ~SimpleVectorialSyncFileFullWaveformWriter() = default;
};

}
}
