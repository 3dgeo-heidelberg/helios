#pragma once

#include <helios/filems/write/comps/SimpleVectorialSyncFileFullWaveformWriter.h>
#include <helios/filems/write/comps/ZipVectorialSyncFileFullWaveformWriter.h>
#include <helios/filems/write/core/BaseFullWaveformWriter.h>
#include <helios/scanner/detector/FullWaveform.h>
#include <helios/util/HeliosException.h>

#include <memory>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Common implementation for any vectorial full waveform writer
 */
class VectorialFullWaveformWriter
  : public BaseFullWaveformWriter<std::vector<FullWaveform> const&>
{
protected:
  // ***  USING  *** //
  // *************** //
  using BaseFullWaveformWriter<std::vector<FullWaveform> const&>::writers;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for vectorial full waveform writer
   */
  VectorialFullWaveformWriter()
    : BaseFullWaveformWriter<std::vector<FullWaveform> const&>()
  {
  }
  virtual ~VectorialFullWaveformWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Writer a vector of full waveforms
   * @param fullWaveForms Vector of full waveforms to be written
   */
  void writeFullWaveforms(std::vector<FullWaveform> const& fullWaveForms);
  /**
   * @brief Like filems::VectorialFullWaveformWriter::writeFullWaveforms but
   *  faster because there is no validation
   * @see filems::VectorialFullWaveformWriter::writeFullWaveforms
   */
  inline void writeFullWaveformsUnsafe(
    std::vector<FullWaveform> const& fullWaveforms) const
  {
    sfw->write(fullWaveforms);
  }
  /**
   * @brief Make a vectorial full waveform SyncFileWriter
   * @see BaseFullWaveformWriter::makeWriter
   */
  std::shared_ptr<SyncFileWriter<std::vector<FullWaveform> const&>> makeWriter(
    std::string const& path) const override
  {
    if (isZipOutput()) {
      return std::make_shared<ZipVectorialSyncFileFullWaveformWriter>(path);
    } else
      return std::make_shared<SimpleVectorialSyncFileFullWaveformWriter>(path);
  }
};

}
}
