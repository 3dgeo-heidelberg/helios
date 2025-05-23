#pragma once

#include <filems/util/FileUtils.h>
#include <filems/write/core/VectorialMeasurementWriter.h>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Common implementation for any vectorial measurement writer that
 *  supports concurrent multiple output streams of the same time
 */
class MultiVectorialMeasurementWriter : public VectorialMeasurementWriter
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for multi vectorial measurement writer
   */
  MultiVectorialMeasurementWriter()
    : VectorialMeasurementWriter()
  {
  }
  ~MultiVectorialMeasurementWriter() override = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Make a multi vectorial measurement SyncFileWriter
   * @see VectorialMeasurementWriter::makeWriter
   */
  std::shared_ptr<
    SyncFileWriter<std::vector<Measurement> const&, glm::dvec3 const&>>
  makeWriter(WriterType const& type,
             std::string const& path,
             bool const zipOutput,
             double const lasScale,
             glm::dvec3 shift,
             double const minIntensity,
             double const deltaIntensity) const override
  {
    // Extract path without extension, and the extension itself
    std::string ext;
    std::string pathNonExt;
    FileUtils::extractExtensionAndPathWithoutExtension(path, ext, pathNonExt);
    // Build vectors of data to build multi vectorial writer
    std::vector<std::string> paths;
    std::vector<double> scaleFactors;
    std::vector<glm::dvec3> shifts;
    std::vector<double> minIntensities;
    std::vector<double> deltaIntensities;
    size_t const nDevs = scanner->getNumDevices();
    for (size_t i = 0; i < nDevs; ++i) {
      paths.push_back(FileUtils::craftPathWithSuffix(
        pathNonExt, "_dev" + scanner->getDeviceId(i), ext));
      scaleFactors.push_back(lasScale);
      shifts.push_back(shift);
      minIntensities.push_back(0.0);
      deltaIntensities.push_back(1000000.0);
    }
    // Build and return the multi vectorial writer
    return SyncFileMeasurementWriterFactory::makeMultiVectorialWriter(
      type,            // Writer type
      paths,           // Output path
      zipOutput,       // Zip flag
      scaleFactors,    // Scale factor
      shifts,          // Offset
      minIntensities,  // Min intensity
      deltaIntensities // Delta intensity
    );
  }
};

}
}
