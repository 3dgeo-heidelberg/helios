#ifndef _HELIOS_FILEMS_BASE_PULSE_WRITER_H_
#define _HELIOS_FILEMS_BASE_PULSE_WRITER_H_

#include <filems/write/core/HeliosWriter.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace helios {
namespace filems {

using std::shared_ptr;
using std::string;
using std::unordered_map;

/**
 * @author Alberto M. Esmoris PEna
 * @version 1.0
 * @brief Class to handle the writing of pulses as an optional output of
 *  HELIOS++. It provides the basis for the implementation of any
 *  pulse writer.
 * @see filems::MeasurementWriter
 * @see filems::VectorialMeasurementWriter
 */
template<typename... WriteArgs>
class BasePulseWriter : public HeliosWriter<WriteArgs...>
{
protected:
  // ***  USING  *** //
  // *************** //
  using HeliosWriter<WriteArgs...>::sfw;

public:
  using HeliosWriter<WriteArgs...>::isZipOutput;
  using HeliosWriter<WriteArgs...>::getOutputPath;

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Map of writers. This map allows to reuse writers for legs grouped
   *  in the same strip.
   */
  unordered_map<string, shared_ptr<SyncFileWriter<WriteArgs...>>> writers{};

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for base pulse writer
   */
  BasePulseWriter() = default;
  virtual ~BasePulseWriter() = default;

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Configure the output path for the base pulse writer
   * @param parent Path to output directory for pulse files
   * @param prefix Prefix for the name of the output file
   * @param writePulse Flag to specify whether the pulses must be written
   *  (true) or not (false)
   */
  virtual void configure(string const& parent,
                         string const& prefix,
                         bool const writePulse);
  /**
   * @brief Make a SyncFileWriter that is suitable to be used by the base
   *  pulse writer
   * @return SyncFileWriter which is compatible with the base full waveform
   *  writer
   * @see SyncFileWriter
   * @see helios::filems::PulseWriter::makeWriter
   * @see helios::filems::VectorialFullWaveformWriter::makeWriter
   */
  virtual shared_ptr<SyncFileWriter<WriteArgs...>> makeWriter(
    string const& path) const = 0;

  // ***  HELIOS WRITER METHODS  *** //
  // ******************************* //
  /**
   * @brief Finishes all writers
   * @see filems::BasePulseWriter::writers
   */
  void finish() override;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Set the path to the output file
   * @param path New path to the output file
   */
  virtual void setOutputFilePath(string const& path);
};

#include <filems/write/core/BasePulseWriter.tpp>

}
}

#endif
