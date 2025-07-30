#pragma once

#include <filems/factory/SyncFileMeasurementWriterFactory.h>
#include <filems/write/core/BaseMeasurementWriter.h>
#include <scanner/Measurement.h>
#include <util/HeliosException.h>

#include <memory>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of measurements to generate HELIOS++ output
 *  virtual point clouds
 * @see filems::BaseMeasurementWriter
 */
class MeasurementWriter
  : public BaseMeasurementWriter<Measurement const&, glm::dvec3 const&>
{
protected:
  // ***  USING  *** //
  // *************** //
  using BaseMeasurementWriter<Measurement const&, glm::dvec3 const&>::scanner;
  using BaseMeasurementWriter<Measurement const&, glm::dvec3 const&>::shift;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @bried Default constructor for measurement writer
   */
  MeasurementWriter()
    : BaseMeasurementWriter<Measurement const&, glm::dvec3 const&>()
  {
  }
  virtual ~MeasurementWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Write a measurement
   * @param m Measurement to be written
   */
  void writeMeasurement(Measurement const& m);
  /**
   * @brief Like filems::MeasurementWriter::writeMeasurement but faster
   *  because there is no validation
   * @see filems::MeasurementWriter::writeMeasurement
   */
  inline void writeMeasurementUnsafe(Measurement const& m) const
  {
    sfw->write(m, shift);
  }
  /**
   * @brief Make a single measurement SyncFileWriter
   * @see BaseMeasurementWriter::makeWriter
   */
  std::shared_ptr<SyncFileWriter<Measurement const&, glm::dvec3 const&>>
  makeWriter(WriterType const& type,
             std::string const& path,
             bool const zipOutput,
             double const lasScale,
             glm::dvec3 shift,
             double const minIntensity,
             double const deltaIntensity) const override
  {
    return SyncFileMeasurementWriterFactory::makeWriter(
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
