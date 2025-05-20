#pragma once

#include <Measurement.h>
#include <filems/write/comps/LasSyncFileWriter.h>
#include <filems/write/strategies/LasMeasurementWriteStrategy.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>

#include <glm/glm.hpp>

#include <memory>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing LasSyncFileWriter to write a vector of
 *  measurements to a LAS14 file.
 *
 * @see filems::LasSyncFileWriter
 * @see filems::LasMeasurementWriteStrategy
 * @see Measurement
 * @see Las14SyncFileMeasurementWriter
 */
class Las14VectorialSyncFileMeasurementWriter
  : public LasSyncFileWriter<vector<Measurement> const&, glm::dvec3 const&>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The measurement write strategy that is wrapped by the main write
   *  strategy in a vectorial fashion
   *  ( filems::LasSyncFileWriter::writeStrategy )
   * @see filems::LasMeasurementWriteStrategy
   */
  LasMeasurementWriteStrategy lmws;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief LAS-1.4 vectorial synchronous file measurement vector writer
   *  constructor
   * @see filems::LasSyncFileWriter::LasSyncFileWriter
   */
  explicit Las14VectorialSyncFileMeasurementWriter(
    const std::string& path,
    bool const compress = false,
    double const scaleFactor = 0.0001,
    glm::dvec3 const offset = glm::dvec3(0, 0, 0),
    double const minIntensity = 0.0,
    double const deltaIntensity = 1000000.0,
    bool const createWriter = true)
    : LasSyncFileWriter<vector<Measurement> const&, glm::dvec3 const&>(
        path,
        compress,
        scaleFactor,
        offset,
        minIntensity,
        deltaIntensity,
        false)
    , lmws(*lw,
           lws.lp,
           lws.scaleFactorInverse,
           lws.offset,
           lws.minIntensity,
           lws.maxIntensity,
           lws.intensityCoefficient,
           lws.ewAttrStart,
           lws.fwiAttrStart,
           lws.hoiAttrStart,
           lws.ampAttrStart)
  {
    // If construct requires creating the wi
    if (createWriter) {
      // Create the LASWriter
      createLasWriter(path, compress);
      // In-place update LasMeasurementWriteStrategy
      new (&lmws) LasMeasurementWriteStrategy(*lw,
                                              lws.lp,
                                              lws.scaleFactorInverse,
                                              lws.offset,
                                              lws.minIntensity,
                                              lws.maxIntensity,
                                              lws.intensityCoefficient,
                                              lws.ewAttrStart,
                                              lws.fwiAttrStart,
                                              lws.hoiAttrStart,
                                              lws.ampAttrStart);
    }
    // Write strategy
    this->writeStrategy =
      std::make_shared<VectorialWriteStrategy<Measurement, glm::dvec3 const&>>(
        lmws);
  }

  // ***  CREATE WRITER  *** //
  // *********************** //
  /**
   * @brief Assist the LasSyncFileWriter::createLasWriters method by
   *  crafting the given specification using the 1.4 version
   * @param lws The LAS write specification to be crafted according to 1.4
   *  version
   */
  void craftSpec(LasWriterSpec& lws) override { lws.craft14(); }
};

}
}
