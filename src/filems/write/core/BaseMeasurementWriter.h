#ifndef _HELIOS_FILEMS_BASE_MEASUREMENT_WRITER_H_
#define _HELIOS_FILEMS_BASE_MEASUREMENT_WRITER_H_

#include <filems/factory/SyncFileMeasurementWriterFactory.h>
#include <filems/write/core/HeliosWriter.h>
#include <scanner/Scanner.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of measurements to generate HELIOS++ output
 *  virtual point clouds. It provides the basis for the implementation of any
 *  measurement writer
 * @see filems::MeasurementWriter
 * @see filems::VectorialMeasurementWriter
 */
template<typename... WriteArgs>
class BaseMeasurementWriter : public HeliosWriter<WriteArgs...>
{
protected:
  // ***  USING  *** //
  // *************** //
  using HeliosWriter<WriteArgs...>::sfw;

public:
  using HeliosWriter<WriteArgs...>::isZipOutput;
  using HeliosWriter<WriteArgs...>::isLasOutput;
  using HeliosWriter<WriteArgs...>::isLas10;
  using HeliosWriter<WriteArgs...>::getLasScale;
  using HeliosWriter<WriteArgs...>::getOutputPath;

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The scanner that generates the measurements to be written
   * @see filems::MeasurementWriter::setScanner
   */
  std::shared_ptr<Scanner> scanner = nullptr;
  /**
   * @brief The pointer to the shift vector to be applied to measurements
   * @see filems::MeasurementWriter::setScanner
   */
  glm::dvec3 shift;

  /**
   * @brief Map of writers. This map allows to reuse writers for legs grouped
   * in the same strip.
   */
  std::unordered_map<std::string, std::shared_ptr<SyncFileWriter<WriteArgs...>>>
    writers{};

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for base measurement writer
   */
  BaseMeasurementWriter() = default;
  virtual ~BaseMeasurementWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Configure the output path for the base measurement writer
   * @param parent Path to output directory for measurements files
   * @param prefix Prefix for the name of the output file
   * @param lastLegInStrip Specify whether the last leg belonged to a strip
   *  (true) or not (false)
   */
  virtual void configure(std::string const& parent,
                         std::string const& prefix,
                         bool const lastLegInStrip);
  /**
   * @brief Choose a type of file writer based on input flags
   * @return Type of writer to be created
   */
  virtual WriterType chooseWriterType() const;
  /**
   * @brief Clear point cloud file for current leg
   */
  virtual void clearPointcloudFile();
  /**
   * @brief Make a SyncFileWriter that is suitable to be used by the base
   *  measurement writer
   * @return SyncFileWriter which is compatible with the base measurement
   *  writer
   * @see SyncFileWriter
   * @see SyncFileMeasurementWriterFactory::makeWriter
   * @see SyncFileMeasurementWriterFactory::makeVectorialWriter
   */
  virtual std::shared_ptr<SyncFileWriter<WriteArgs...>> makeWriter(
    WriterType const& type,
    std::string const& path,
    bool const zipOutput,
    double const lasScale,
    glm::dvec3 shift,
    double const minIntensity,
    double const deltaIntensity) const = 0;

  // ***  HELIOS WRITER METHODS  *** //
  // ******************************* //
  /**
   * @brief Finishes all writers
   * @see filems::BaseMeasurementWriter::writers
   * @see HeliosWriter::finish
   */
  void finish() override;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Set path to output file
   * @param path New path to output file
   */
  virtual void setOutputFilePath(std::string const& path,
                                 bool const lastLegInStrip);
  /**
   * @brief Obtain the scanner associated with the base measurement writer
   * @see filems::BaseMeasurementWriter::scanner
   */
  inline std::shared_ptr<Scanner> getScanner() const { return scanner; }
  /**
   * @brief Associate a new scanner with the base measurement writer, which
   *  implies updating the shift vector to be the same than the one
   *  defined for the scene associated to the scanner
   * @see filems::BaseMeasurementWriter::scanner
   * @see filems::BaseMeasurementWriter::shift
   */
  inline void setScanner(std::shared_ptr<Scanner> scanner)
  {
    this->scanner = scanner;
    if (scanner != nullptr) {
      this->shift = this->scanner->platform->scene->getShift();
    }
  }
  /**
   * @brief Obtain the shift applied by the base measurement writer
   * @see filems::BaseMeasurementWriter::shift
   */
  inline glm::dvec3 const& getShift() { return shift; }
};

#include <filems/write/core/BaseMeasurementWriter.tpp>

}
}

#endif
