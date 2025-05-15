#pragma once

#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>
#include <filems/write/strategies/ZipMeasurementWriteStrategy.h>

#include <memory>
#include <vector>

namespace helios {
namespace filems {

using std::make_shared;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipSyncFileWriter to write a vector of
 *  measurements to a zip file.
 *
 * @see filems::ZipSyncFileWriter
 * @see filems::ZipMeasurementWriteStrategy
 * @see Measurement
 * @see filems::ZipSyncFileMeasurementWriter
 */
class ZipVectorialSyncFileMeasurementWriter
  : public ZipSyncFileWriter<vector<Measurement> const&, glm::dvec3 const&>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The measurement write strategy that is wrapped by the main write
   *  strategy in a vectorial fashion
   *  ( filems::ZipSyncFileWriter::writeStrategy )
   * @see filems::ZipMeasurementWriteStrategy
   */
  ZipMeasurementWriteStrategy zmws;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief ZIP synchronous file measurement vector writer constructor
   * @see filems::ZipSyncFileWriter::ZipSyncFileWriter
   */
  explicit ZipVectorialSyncFileMeasurementWriter(
    const string& path,
    int compressionMode = boost::iostreams::zlib::best_compression)
    : ZipSyncFileWriter<vector<Measurement> const&, glm::dvec3 const&>(
        path,
        compressionMode)
    , zmws(this->ofs, *(this->oa))
  {
    this->writeStrategy =
      make_shared<VectorialWriteStrategy<Measurement, glm::dvec3 const&>>(zmws);
  }
  virtual ~ZipVectorialSyncFileMeasurementWriter() = default;
};

}
}
