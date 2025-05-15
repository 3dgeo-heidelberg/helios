#pragma once

#include <filems/write/comps/SimpleVectorialSyncFileFullWaveformWriter.h>
#include <filems/write/comps/ZipVectorialSyncFileFullWaveformWriter.h>
#include <filems/write/core/BaseFullWaveformWriter.h>
#include <scanner/detector/FullWaveform.h>
#include <util/HeliosException.h>

#include <memory>
#include <string>

namespace helios {
namespace filems {

using std::shared_ptr;
using std::string;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Common implementation for any vectorial full waveform writer
 */
class VectorialFullWaveformWriter
  : public BaseFullWaveformWriter<vector<FullWaveform> const&>
{
protected:
  // ***  USING  *** //
  // *************** //
  using BaseFullWaveformWriter<vector<FullWaveform> const&>::writers;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for vectorial full waveform writer
   */
  VectorialFullWaveformWriter()
    : BaseFullWaveformWriter<vector<FullWaveform> const&>()
  {
  }
  virtual ~VectorialFullWaveformWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Writer a vector of full waveforms
   * @param fullWaveforms Vector of full waveforms to be written
   */
  void writeFullWaveforms(vector<FullWaveform> const& fullWaveforms);
  /**
   * @brief Like filems::VectorialFullWaveformWriter::writeFullWaveforms but
   *  faster because there is no validation
   * @see filems::VectorialFullWaveformWriter::writeFullWaveforms
   */
  inline void writeFullWaveformsUnsafe(
    vector<FullWaveform> const& fullWaveforms) const
  {
    sfw->write(fullWaveforms);
  }
  /**
   * @brief Make a vectorial full waveform SyncFileWriter
   * @see BaseFullWaveformWriter::makeWriter
   */
  shared_ptr<SyncFileWriter<std::vector<FullWaveform> const&>> makeWriter(
    string const& path) const override
  {
    if (isZipOutput()) {
      return make_shared<ZipVectorialSyncFileFullWaveformWriter>(path);
    } else
      return make_shared<SimpleVectorialSyncFileFullWaveformWriter>(path);
  }
};

}
}
