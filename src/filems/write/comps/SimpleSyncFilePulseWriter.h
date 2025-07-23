#pragma once

#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/strategies/DirectPulseWriteStrategy.h>
#include <scanner/PulseRecord.h>

#include <glm/glm.hpp>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write pulse
 *  data directly to a file.
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectPulseWriteStrategy
 */
class SimpleSyncFilePulseWriter
  : public SimpleSyncFileWriter<PulseRecord const&>
{

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous file pulse writer constructor
   * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
   */
  explicit SimpleSyncFilePulseWriter(
    const std::string& path,
    std::ios_base::openmode om = std::ios_base::app)
    : SimpleSyncFileWriter<PulseRecord const&>(path, om)
  {
    this->writeStrategy = std::make_shared<DirectPulseWriteStrategy>(this->ofs);
  }
  virtual ~SimpleSyncFilePulseWriter() = default;
};

}
}
