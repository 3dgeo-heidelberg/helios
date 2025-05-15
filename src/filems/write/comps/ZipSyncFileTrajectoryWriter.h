#pragma once

#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/strategies/ZipTrajectoryWriteStrategy.h>

#include <memory>

namespace helios {
namespace filems {

using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipSyncFileWriter to write
 *  compressed trajectory to a file
 */
class ZipSyncFileTrajectoryWriter : public ZipSyncFileWriter<Trajectory const&>
{

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous zipped trajectory writer constructor
   * @see filems::ZipSyncFileWriter::ZipSyncFileWriter
   */
  explicit ZipSyncFileTrajectoryWriter(
    string const& path,
    int compressionMode = boost::iostreams::zlib::best_compression)
    : ZipSyncFileWriter<Trajectory const&>(path, compressionMode)
  {
    this->writeStrategy =
      make_shared<ZipTrajectoryWriteStrategy>(this->ofs, *(this->oa));
  }
  virtual ~ZipSyncFileTrajectoryWriter() = default;
};

}
}
