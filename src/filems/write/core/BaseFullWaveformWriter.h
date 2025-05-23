#ifndef _HELIOS_FILEMS_BASE_FULLWAVEFORM_WRITER_H_
#define _HELIOS_FILEMS_BASE_FULLWAVEFORM_WRITER_H_

#include <filems/write/core/HeliosWriter.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace helios {
namespace filems {

template<typename... WriteArgs>
class BaseFullWaveformWriter : public HeliosWriter<WriteArgs...>
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
   * @brief Map of writers. This map allows to reuse writers for legs grouped
   *  in the same strip.
   */
  std::unordered_map<std::string, std::shared_ptr<SyncFileWriter<WriteArgs...>>>
    writers{};

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for base full waveform writer
   */
  BaseFullWaveformWriter() = default;
  virtual ~BaseFullWaveformWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Configure the output path for the full waveform writer
   * @param parent Path to output directory for full waveform files
   * @param prefix Prefix for the name of the output file
   * @param computeWaveform Flag to specify if waveform must be computed
   *  (true) or not (false)
   */
  virtual void configure(std::string const& parent,
                         std::string const& prefix,
                         bool const computeWaveform);
  /**
   * @brief Make a SyncFileWriter that is suitable to be used by the base
   *  full waveform writer
   * @return SyncFileWriter which is compatible with the base full waveform
   *  writer
   * @see SyncFileWriter
   * @see helios::filems::FullWaveformWriter::makeWriter
   * @see helios::filems::VectorialFullWaveformWriter::makeWriter
   */
  virtual std::shared_ptr<SyncFileWriter<WriteArgs...>> makeWriter(
    std::string const& path) const = 0;

  // ***  HELIOS WRITER METHODS  *** //
  // ******************************* //
  /**
   * @brief Finishes all writers
   * @see filems::BaseFullWaveformWriter::writers
   * @see HeliosWriter::finish
   */
  void finish() override;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Set the path to output file
   * @param path New path to output file
   */
  void setOutputFilePath(std::string const& path);
  /**
   * @brief Get the path to the output file
   * @return The path to the output file
   * @see filems::BaseFullWaveformWriter::getOutputPath
   */
  fs::path getOutputFilePath() const override
  {
    return fs::path(getOutputPath());
  }
  /**
   * @see filems::BaseFullWaveformWriter::getOutputFilePath
   */
  std::string getOutputPath() const override { return sfw->getPath(); }
};

#include <filems/write/core/BaseFullWaveformWriter.tpp>

}
}

#endif
