#pragma once

#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/strategies/ZipMeasurementWriteStrategy.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipSyncFileWriter to write
 *  compressed measurements to a file
 *
 * @see filems::ZipSyncFileWriter
 * @see filems::ZipM̀easurementWriteStrategy
 * @see Measurement
 */
class ZipSyncFileMeasurementWriter
  : public ZipSyncFileWriter<Measurement const&, glm::dvec3 const&>
{

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous zipped measurement writer constructor
   * @see filems::ZipSyncFileWriter::ZipSyncFileWriter
   */
  explicit ZipSyncFileMeasurementWriter(
    const std::string& path,
    int compressionMode = boost::iostreams::zlib::best_compression)
    : ZipSyncFileWriter<Measurement const&, glm::dvec3 const&>(path,
                                                               compressionMode)
  {
    this->writeStrategy =
      std::make_shared<ZipMeasurementWriteStrategy>(this->ofs, *(this->oa));
  }
  virtual ~ZipSyncFileMeasurementWriter() = default;
};

}
}
