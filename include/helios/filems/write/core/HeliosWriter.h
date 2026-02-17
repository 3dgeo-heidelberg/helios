#ifndef _HELIOS_FILEMS_HELIOS_WRITER_H_
#define _HELIOS_FILEMS_HELIOS_WRITER_H_
#include <helios/filems/write/comps/SyncFileWriter.h>
#include <helios/util/HeliosException.h>

#include <boost/filesystem.hpp>

#include <memory>
#include <string>

namespace fs = boost::filesystem;

namespace helios {
namespace filems {

/**
 * @brief Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Base class providing the core for writers of different HELIOS++
 *  outputs (measurements, trajectory, full waveform)
 */
template<typename... WriteArgs>
class HeliosWriter
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Synchronous file writer
   * @see filems::SyncFileWriter
   */
  std::shared_ptr<SyncFileWriter<WriteArgs...>> sfw = nullptr;

  /**
   * @brief Flag specifying if detector output must be written in LAS
   * format (true) or not (false)
   * @see helios::filems::HeliosWriter::lasScale
   */
  bool lasOutput = false;
  /**
   * @brief Flag specifying if detect output must be writing in LAS 1.0
   * (LAS 1.4 is written by default)
   */
  bool las10 = false;
  /**
   * @brief Flag specifying if detector output must be zipped (true)
   * or not (false)
   */
  bool zipOutput = false;
  /**
   * @brief Scale factor specification to be used when LAS output format
   * specified
   * @see helios::filems::HeliosWriter::lasOutput
   */
  double lasScale = 0.0001;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for HELIOS++ writer
   */
  HeliosWriter() = default;
  virtual ~HeliosWriter() = default;

  // ***  HELIOS WRITER METHODS  *** //
  // ******************************* //
  /**
   * @brief Finish the sync file writer
   * @see filems::SyncFileWriter::finish
   */
  virtual void finish();

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Check whether the HeliosWriter has a non null writer (true) or
   *  not (false)
   * @return True if the HeliosWriter has a non null writer, false otherwise
   */
  virtual bool hasWriter() const { return sfw != nullptr; }
  /**
   * @brief Get the path to the output file
   * @return The path to the output file
   * @see filems::HeliosWriter::getOutputPath
   */
  virtual fs::path getOutputFilePath() const
  {
    return fs::path(getOutputPath());
  }
  /**
   * @see filems::HeliosWriter::getOutputFilePath
   */
  virtual std::string getOutputPath() const { return sfw->getPath(); }

  /**
   * @brief Get the LAS output flag
   * @see filems::HeliosWriter::lasOutput
   */
  inline bool isLasOutput() const { return lasOutput; }
  /**
   * @brief Set the LAS output flag
   * @see filems::HeliosWriter::lasOutput
   */
  inline void setLasOutput(bool const lasOutput)
  {
    this->lasOutput = lasOutput;
  }
  /**
   * @brief Get the LAS 10 specification flag
   * @see filems::HeliosWriter::las10
   */
  inline bool isLas10() const { return las10; }
  /**
   * @brief Set the LAS output flag
   * @see filems::HeliosWriter::las10
   */
  inline void setLas10(bool const las10) { this->las10 = las10; }
  /**
   * @brief Get the zip output flag
   * @see filems::HeliosWriter::zipOutput
   */
  inline bool isZipOutput() const { return zipOutput; }
  /**
   * @brief Set the zip output flag
   * @see filems::HeliosWriter::zipOutput
   */
  inline void setZipOutput(bool const zipOutput)
  {
    this->zipOutput = zipOutput;
  }
  /**
   * @brief Obtain the LAS scale of the measurement writer
   * @see filems::HeliosWriter::lasScale
   */
  inline double getLasScale() const { return lasScale; }
  /**
   * @brief Set the LAS scale of the measurement writer
   * @see filems::HeliosWriter::lasScale
   */
  inline void setLasScale(double const lasScale) { this->lasScale = lasScale; }
};

#include <helios/filems/write/core/HeliosWriter.tpp>

}
}

#endif
