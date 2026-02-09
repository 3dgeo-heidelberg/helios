#pragma once

#include <filems/factory/SyncFileMeasurementWriterFactory.h>
#include <filems/write/core/BaseMeasurementWriter.h>
#include <util/HeliosException.h>

#include <memory>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Common implementation for any vectorial measurement writer
 */
class VectorialMeasurementWriter
  : public BaseMeasurementWriter<std::vector<Measurement> const&,
                                 glm::dvec3 const&>
{
protected:
  // ***  USING  *** //
  // *************** //
  using BaseMeasurementWriter<std::vector<Measurement> const&,
                              glm::dvec3 const&>::scanner;
  using BaseMeasurementWriter<std::vector<Measurement> const&,
                              glm::dvec3 const&>::shift;
  using BaseMeasurementWriter<std::vector<Measurement> const&,
                              glm::dvec3 const&>::writers;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for vectorial measurement writer
   */
  VectorialMeasurementWriter()
    : BaseMeasurementWriter<std::vector<Measurement> const&,
                            glm::dvec3 const&>()
  {
  }
  virtual ~VectorialMeasurementWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Write a vector of measurements
   * @param measurements Vector of measurements to be written
   */
  void writeMeasurements(std::vector<Measurement> const& measurements);
  /**
   * @brief Like filems::VectorialMeasurementWriter::writeMeasurements but
   *  faster because there is no validation
   * @see filems::VectorialMeasurementWriter::writeMeasurements
   */
  inline void writeMeasurementsUnsafe(
    std::vector<Measurement> const& measurements) const
  {
    sfw->write(measurements, shift);
  }
  /**
   * @brief Make a vectorial measurement SyncFileWriter
   * @see BaseMeasurementWriter::makeWriter
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
    return SyncFileMeasurementWriterFactory::makeVectorialWriter(
      type,      // Writer type
      path,      // Output path
      zipOutput, // Zip flag
      lasScale,  // Scale factor
      shift,     // Offset
      0.0,       // Min intensity
      1000000.0  // Delta intensity
    );
  }
};

}
}
