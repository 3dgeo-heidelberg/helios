//
// Created by miguelyermo on 18/5/21.
//

/*
 * FILENAME :  SyncFileMeasurementWriterFactory.h
 * PROJECT  :  helios
 * DESCRIPTION :
 *
 *
 *
 *
 *
 * AUTHOR :    Miguel Yermo        START DATE : 16:34 18/5/21
 *
 */

#pragma once

#include <helios/filems/write/comps/Las14MultiVectorialSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/Las14SyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/Las14VectorialSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/LasMultiVectorialSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/LasSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/LasVectorialSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/SimpleMultiVectorialSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/SimpleSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/SimpleVectorialSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/SyncFileWriter.h>
#include <helios/filems/write/comps/ZipMultiVectorialSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/ZipSyncFileMeasurementWriter.h>
#include <helios/filems/write/comps/ZipVectorialSyncFileMeasurementWriter.h>
#include <helios/util/HeliosException.h>

#include <memory>
#include <sstream>
#include <string>

namespace helios {
namespace filems {

// ** Types of writers ** //

/** @enum WriterType
 *  @brief Different types of SyncFileWriter to be created
 */
enum WriterType
{
  las10Type, /**< LAS v1.0 file writer type */
  las14Type, /**< LAS v1.4 file writer type */
  zipType,   /**< Zipped text file writer type */
  simpleType /**< .xyz file writer type */
};

// ** SyncFileWriter factory ** //

/**
 * @author Miguel Yermo Garcia
 * @version 1.0
 * @brief SyncFileMeasurementWriter Factory class. Used to create the
 *  appropriate measurement writers based on input flags
 */
class SyncFileMeasurementWriterFactory
{

public:
  // ***  STATIC MAKE METHODS  *** //
  // ***************************** //
  /**
   * @brief Synchronous file writer factory
   * @see SyncFileMeasurementWriterFactory::WriterType
   * @see SyncFileWriter::path
   * @param compress Specify is use compressed LAS format (LAZ) or not (pure
   *  LAS)
   * @see LasSyncFileWriter::scaleFactor
   * @see LasSyncFileWriter::offset
   * @see LasSyncFileWriter::minIntensity
   * @see LasSyncFileWriter::deltaIntensity
   */
  static std::shared_ptr<SyncFileWriter<Measurement const&, glm::dvec3 const&>>
  makeWriter(WriterType const type,
             const std::string& path,
             bool const compress = false,
             double const scaleFactor = 0.0001,
             glm::dvec3 const offset = glm::dvec3(0, 0, 0),
             double const minIntensity = 0.0,
             double const deltaIntensity = 1000000.0)
  {
    switch (type) {
      case las10Type:
        return std::make_shared<LasSyncFileMeasurementWriter>(
          path,          // Output path
          compress,      // Zip flag
          scaleFactor,   // Scale factor
          offset,        // Offset
          minIntensity,  // Min intensity
          deltaIntensity // Delta intensity
        );
      case las14Type:
        return std::make_shared<Las14SyncFileMeasurementWriter>(
          path,          // Output path
          compress,      // Zip flag
          scaleFactor,   // Scale factor
          offset,        // Offset
          minIntensity,  // Min intensity
          deltaIntensity // Delta intensity
        );
      case zipType:
        return std::make_shared<ZipSyncFileMeasurementWriter>(path);
      case simpleType:
        return std::make_shared<SimpleSyncFileMeasurementWriter>(path);
    }

    // Handle unexpected type
    std::stringstream ss;
    ss << "SyncFileMeasurementWriterFactory::makeWriter received an "
       << "unexpected type: (" << type << ")";
    throw HeliosException(ss.str());
  }

  /**
   * @brief Synchronous vectorial file writer factory
   * @see SyncFileMeasurementWriterFactory::WriterType
   * @see SyncFileWriter::path
   * @param compress Specify is use compressed LAS format (LAZ) or not (pure
   *  LAS)
   * @see LasSyncFileWriter::scaleFactor
   * @see LasSyncFileWriter::offset
   * @see LasSyncFileWriter::minIntensity
   * @see LasSyncFileWriter::deltaIntensity
   */
  static std::shared_ptr<
    SyncFileWriter<std::vector<Measurement> const&, glm::dvec3 const&>>
  makeVectorialWriter(WriterType const type,
                      const std::string& path,
                      bool const compress = false,
                      double const scaleFactor = 0.0001,
                      glm::dvec3 const offset = glm::dvec3(0, 0, 0),
                      double const minIntensity = 0.0,
                      double const deltaIntensity = 1000000.0)
  {
    switch (type) {
      case las10Type:
        return std::make_shared<LasVectorialSyncFileMeasurementWriter>(
          path,          // Output path
          compress,      // Zip flag
          scaleFactor,   // Scale factor
          offset,        // Offset
          minIntensity,  // Min intensity
          deltaIntensity // Delta intensity
        );
      case las14Type:
        return std::make_shared<Las14VectorialSyncFileMeasurementWriter>(
          path,          // Output path
          compress,      // Zip flag
          scaleFactor,   // Scale factor
          offset,        // Offset
          minIntensity,  // Min intensity
          deltaIntensity // Delta intensity
        );
      case zipType:
        return std::make_shared<ZipVectorialSyncFileMeasurementWriter>(path);
      case simpleType:
        return std::make_shared<SimpleVectorialSyncFileMeasurementWriter>(path);
    }

    // Handle unexpected type
    std::stringstream ss;
    ss << "SyncFileMeasurementWriterFactory::makeVectorialWriter received an "
       << "unexpected type: (" << type << ")";
    throw HeliosException(ss.str());
  }

  /**
   * @brief Synchronous multi-stream vectorial file writer factory
   * @see SyncFileMeasurementWriterFactory::WriterType
   * @see SyncFileWriter::path
   * @param compress Specify is use compressed LAS format (LAZ) or not (pure
   *  LAS)
   * @see LasSyncFileWriter::scaleFactor
   * @see LasSyncFileWriter::offset
   * @see LasSyncFileWriter::minIntensity
   * @see LasSyncFileWriter::deltaIntensity
   */
  static std::shared_ptr<
    SyncFileWriter<std::vector<Measurement> const&, glm::dvec3 const&>>
  makeMultiVectorialWriter(WriterType const type,
                           std::vector<std::string> const& path,
                           bool const compress,
                           std::vector<double> const& scaleFactor,
                           std::vector<glm::dvec3> const& offset,
                           std::vector<double> const& minIntensity,
                           std::vector<double> const& deltaIntensity)
  {
    switch (type) {
      case las10Type:
        return std::make_shared<LasMultiVectorialSyncFileMeasurementWriter>(
          path,          // Output path
          compress,      // Zip flag
          scaleFactor,   // Scale factor
          offset,        // Offset
          minIntensity,  // Min intensity
          deltaIntensity // Delta intensity
        );
      case las14Type:
        return std::make_shared<Las14MultiVectorialSyncFileMeasurementWriter>(
          path,          // Output path
          compress,      // Zip flag
          scaleFactor,   // Scale factor
          offset,        // Offset
          minIntensity,  // Min intensity
          deltaIntensity // Delta intensity
        );
      case zipType:
        return std::make_shared<ZipMultiVectorialSyncFileMeasurementWriter>(
          path);
      case simpleType:
        return std::make_shared<SimpleMultiVectorialSyncFileMeasurementWriter>(
          path);
    }

    // Handle unexpected type
    std::stringstream ss;
    ss << "SyncFileMeasurementWriterFactory::makeMultiVectorialWriter "
       << "received an unexpected type: (" << type << ")";
    throw HeliosException(ss.str());
  }
};

}
}
