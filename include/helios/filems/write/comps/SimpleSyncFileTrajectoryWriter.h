#pragma once

#include <helios/filems/write/comps/SimpleSyncFileTrajectoryWriter.h>
#include <helios/filems/write/comps/SimpleSyncFileWriter.h>
#include <helios/filems/write/strategies/DirectTrajectoryWriteStrategy.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write
 *  trajectory directly to a file.
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectTrajectoryWriteStrategy
 * @see Trajectory
 */
class SimpleSyncFileTrajectoryWriter
  : public SimpleSyncFileWriter<Trajectory const&>
{

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous file trajectory writer constructor
   * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
   */
  explicit SimpleSyncFileTrajectoryWriter(
    const std::string& path,
    std::ios_base::openmode om = std::ios_base::app)
    : SimpleSyncFileWriter<Trajectory const&>(path, om)
  {
    this->writeStrategy =
      std::make_shared<DirectTrajectoryWriteStrategy>(this->ofs);
  }
  virtual ~SimpleSyncFileTrajectoryWriter() = default;
};

}
}
