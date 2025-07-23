#pragma once

#include <filems/write/comps/LasMultiVectorialSyncFileMeasurementWriter.h>
#include <filems/write/comps/MultiLasSyncFileWriter.h>

#include <memory>
#include <vector>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing MultiLasSyncFileWriter to write vectors
 *  of measurements to LAS-1.4 files supporting concurrent multiple output
 *  streams
 *
 * @see filems::MultiLasSyncFileWriter
 * @see filems::LasMeasurementWriteStrategy
 * @see Measurement
 * @see LasSyncFileMeasurementWriter
 */
class Las14MultiVectorialSyncFileMeasurementWriter
  : public LasMultiVectorialSyncFileMeasurementWriter
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The measurement write strategies that are wrapped by the main
   *  write strategies in a vectorial fashion
   */
  std::vector<LasMeasurementWriteStrategy> lmws;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief LAS-1.4 multi-vecctorial synchronous file measurement vector
   *  writer
   * @see filems::MultiLasSyncFileWriter::MultiLasSyncFileWriter
   */
  explicit Las14MultiVectorialSyncFileMeasurementWriter(
    std::vector<std::string> const& path,
    bool const compress,
    std::vector<double> const& scaleFactor,
    std::vector<glm::dvec3> const& offset,
    std::vector<double> const& minIntensity,
    std::vector<double> const& deltaIntensity,
    bool const createWriter = true)
    : LasMultiVectorialSyncFileMeasurementWriter(path,
                                                 compress,
                                                 scaleFactor,
                                                 offset,
                                                 minIntensity,
                                                 deltaIntensity,
                                                 false)
  {
    // If construct must create the writers
    if (createWriter) {
      // Create each LASWriter
      createLasWriters(path, compress);
    }
    // Build measurement write strategies
    buildMeasurementWriteStrategies();
    // Build vectorial write strategies
    // WARNING : It must be done after building the measurement write
    // strategies to be wrapped by the vectorial strategy. If the vector
    // of measurement strategies is modified, then the references in the
    // vectorial strategy objects will be inconsistent.
    buildVectorialWriteStrategies();
  }
  virtual ~Las14MultiVectorialSyncFileMeasurementWriter() = default;

  // ***  CREATE WRITER  *** //
  // *********************** //
  /**
   * @brief Assist the MultiLasSyncFileWriter::createLasWriters method by
   *  crafting the given specification using the 1.4 version
   * @param lws The LAS write specification to be crafted according to 1.4
   *  version
   */
  void craftSpec(LasWriterSpec& lws) override { lws.craft14(); };
};

}
}
