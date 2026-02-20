#pragma once

#include <filems/write/comps/SimpleVectorialSyncFilePulseWriter.h>
#include <filems/write/core/BasePulseWriter.h>
#include <scanner/PulseRecord.h>
#include <util/HeliosException.h>

#include <glm/glm.hpp>

#include <memory>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Common implementation for any vectorial pulse writer
 */
class VectorialPulseWriter
  : public BasePulseWriter<std::vector<PulseRecord> const&>
{
protected:
  // ***  USING  *** //
  // *************** //
  using BasePulseWriter<std::vector<PulseRecord> const&>::writers;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for vectorial pulse writer
   */
  VectorialPulseWriter()
    : BasePulseWriter<std::vector<PulseRecord> const&>()
  {
  }
  virtual ~VectorialPulseWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Writer a vector of pulses (represented by many vectors of the
   *  same dimensionality with one component per pulse).
   */
  void writePulses(std::vector<PulseRecord> const& pulseRecords);
  /**
   * @brief Like filems::VectorialPulseWriter::writePulses but faster because
   *  there is no validation
   * @see filems::VectorialPulseWriter::writePulses
   */
  inline void writePulsesUnsafe(std::vector<PulseRecord> const& pulseRecords)
  {
    sfw->write(pulseRecords);
  }
  /**
   * @brief Make a vectorial pulse SyncFileWriter
   * @see BasePulseWriter::makeWriter
   */
  std::shared_ptr<SyncFileWriter<std::vector<PulseRecord> const&>> makeWriter(
    std::string const& path) const override
  {
    return std::make_shared<SimpleVectorialSyncFilePulseWriter>(path);
  }
};

}
}
