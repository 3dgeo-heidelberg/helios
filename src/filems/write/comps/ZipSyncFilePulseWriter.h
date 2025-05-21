#pragma once

#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/strategies/ZipPulseWriteStrategy.h>
#include <scanner/PulseRecord.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipSyncFileWriter to write compressed
 *  pulse data to a file
 *
 * @see filems::ZipSyncFileWriter
 * @see filems::ZipSyncFilePulseWriter
 */
class ZipSyncFilePulseWriter : public ZipSyncFileWriter<PulseRecord const&>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous zipped pulse writer constructor
   * @see filems::ZipSyncFileWriter::ZipSyncFileWriter
   */
  explicit ZipSyncFilePulseWriter(
    std::string const& path,
    int compressionMode = boost::iostreams::zlib::best_compression)
    : ZipSyncFileWriter<PulseRecord const&>(path, compressionMode)
  {
    this->writeStrategy =
      std::make_shared<ZipPulseWriteStrategy>(this->ofs, *(this->oa));
  }
  virtual ~ZipSyncFilePulseWriter() = default;
};

}
}
