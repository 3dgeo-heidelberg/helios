#pragma once

#include <filems/write/comps/SimpleSyncFileFullWaveformWriter.h>
#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/comps/ZipSyncFileFullWaveformWriter.h>
#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/core/BaseFullWaveformWriter.h>
#include <scanner/detector/FullWaveform.h>

#include <boost/filesystem.hpp>
#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of full waveform to generate HELIOS++ output
 *  virtual full waveform
 */
class FullWaveformWriter : public BaseFullWaveformWriter<FullWaveform const&>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for full waveform writer
   */
  FullWaveformWriter() = default;
  virtual ~FullWaveformWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Write full waveform data
   */
  void writeFullWaveform(FullWaveform const& fullWaveform);
  /**
   * @brief Like filems::FullWaveformWriter::writeFullWaveform but faster
   *  because there is no validation
   * @see filems::FullWaveformWriter::writeFullWaveform
   */
  inline void writeFullWaveformUnsafe(FullWaveform const& fullWaveform) const
  {
    sfw->write(fullWaveform);
  }
  /**
   * @brief Make a single full waveform SyncFileWriter
   * @see BaseFullWaveformWriter::makeWriter
   */
  std::shared_ptr<SyncFileWriter<FullWaveform const&>> makeWriter(
    std::string const& path) const override
  {
    if (isZipOutput()) {
      return std::make_shared<ZipSyncFileFullWaveformWriter>(path);
    } else
      return std::make_shared<SimpleSyncFileFullWaveformWriter>(path);
  }
};

}
}
