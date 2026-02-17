#pragma once

#include <helios/filems/write/comps/SimpleSyncFileWriter.h>
#include <helios/filems/write/strategies/DirectStringWriteStrategy.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write
 *  measurements directly to a file.
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectStringWriteStrategy
 * @see Measurement
 */
class SimpleSyncFileStringWriter
  : public SimpleSyncFileWriter<std::string const&>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous file string writer constructor
   * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
   */
  explicit SimpleSyncFileStringWriter(
    const std::string& path,
    std::ios_base::openmode om = std::ios_base::app)
    : SimpleSyncFileWriter<std::string const&>(path, om)
  {
    this->writeStrategy =
      std::make_shared<DirectStringWriteStrategy>(this->ofs);
  }
  virtual ~SimpleSyncFileStringWriter() = default;
};

}
}
