#pragma once

#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/strategies/DirectPulseWriteStrategy.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>
#include <scanner/PulseRecord.h>

#include <memory>
#include <vector>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write a vector
 *  of pulse records directly to a file
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectPulseWriteStrategy
 * @see PulseRecord
 * @see filems::SimpleSyncFilePulseWriter
 */
class SimpleVectorialSyncFilePulseWriter
  : public SimpleSyncFileWriter<std::vector<PulseRecord> const&>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The pulse write strategy that is wrapped by the main write
   *  strategy in a vectorial fashion
   *  ( filems::SimpleSyncFileWriter::writeStrategy )
   * @see filems::DirectPulseWriteStrategy
   */
  DirectPulseWriteStrategy dpws;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous file pulse vector writer constructor
   * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
   */
  explicit SimpleVectorialSyncFilePulseWriter(
    const std::string& path,
    std::ios_base::openmode om = std::ios_base::app)
    : SimpleSyncFileWriter<vector<PulseRecord> const&>(path, om)
    , dpws(this->ofs)
  {
    this->writeStrategy =
      std::make_shared<VectorialWriteStrategy<PulseRecord>>(dpws);
  }
  virtual ~SimpleVectorialSyncFilePulseWriter() = default;
};

}
}
