#pragma once

#include <helios/filems/util/LasWriterSpec.h>
#include <helios/filems/write/comps/SingleSyncFileWriter.h>

#include <glm/glm.hpp>
#include <laswriter.hpp>

#include <ctime>
#include <memory>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Abstract specialization of SingleSyncFileWriter to write output in
 *  LAS format
 * @see filems::SingleSyncFileWriter
 */
template<typename... WriteArgs>
class LasSyncFileWriter : public SingleSyncFileWriter<WriteArgs...>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The specification defining the LAS writer
   */
  LasWriterSpec lws;
  /**
   * @brief LASwriter. Used to write to LAS file.
   */
  std::shared_ptr<LASwriter> lw;
  /**
   * @brief Flag used to control the sync writer status
   */
  bool finished;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for Synchronous LAS file writer
   */
  LasSyncFileWriter()
    : SingleSyncFileWriter<WriteArgs...>() {};
  /**
   * @brief Synchronous LAS file writer constructor
   * @see SyncFileWriter::path
   * @see filems::LasWriterSpec::LasWriterSpec
   * @see filems::LasWriterSpec::scaleFactor
   * @see filems::LasWriterSpec::offset
   * @see filems::LasWriterSpec::minIntensity
   * @see filems::LasWriterSpec::deltaIntensity
   * @see filems::LasWriterSpec
   */
  explicit LasSyncFileWriter(const std::string& path,
                             bool const compress = false,
                             double const scaleFactor = 0.0001,
                             glm::dvec3 const offset = glm::dvec3(0, 0, 0),
                             double const minIntensity = 0.0,
                             double const deltaIntensity = 1000000.0,
                             bool const createWriter = true

                             )
    : SingleSyncFileWriter<WriteArgs...>(path)
    , lws(path, scaleFactor, offset, minIntensity, deltaIntensity)
    , finished(false)
  {
    // If construct must create the writer
    if (createWriter) {
      // Create LASWriter
      createLasWriter(path, compress);
    }
  }

  virtual ~LasSyncFileWriter() { LasSyncFileWriter::finish(); }

  // ***  CREATE WRITER  *** //
  // *********************** //
  /**
   * @brief Creation of the LasWriter itself, including LASpoint
   * initialization
   * @param path Path where the file will be save
   * @param compress Flag to activate/deactivate compression (las/laz format)
   */
  virtual void createLasWriter(const std::string& path, bool const compress)
  {
    // Craft header and point format
    craftSpec(lws);

    // Add extra attributes
    lws.addExtraAttributes();

    // Initialize LASpoint
    lws.initLASPoint();

    // Create writer from specification
    lw = lws.makeWriter(path, compress);
  }
  /**
   * @brief Assist the LasSyncFileWriter::createLasWriters method by
   *  crafting the given specification
   * @param lws The LAS write specification to be crafted
   */
  virtual void craftSpec(LasWriterSpec& lws) { lws.craft(); }

  // ***  F I N I S H  *** //
  // ********************* //
  /**
   * @brief LasSyncFileWriter updates header and guarantees writings
   * have been done only after the finish method has been invoked.
   * If it has not been manually invoked, then it will when destroying the
   * instance.
   * Once the finish method has been invoked, the LasSyncFileWriter should
   * not be used again.
   * @see filems::LasWriterSpec::finish
   */
  void finish() override
  {
    if (finished)
      return;                               // Check whether finished or not
    lw->update_header(&lws.lwHeader, true); // Update the writer's header
    lws.finish();    // Finish the initialized specification
    lw->close();     // Close the writer itself
    finished = true; // Flag as finished
  };
};

}
}
