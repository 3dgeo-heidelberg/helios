//
// Created by miguelyermo on 18/5/21.
//

/*
 * FILENAME :  SyncFileWriterFactory.h
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

#include <filems/write/Las14SyncFileWriter.h>
#include <filems/write/LasSyncFileWriter.h>
#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/comps/SyncFileWriter.h>
#include <filems/write/comps/ZipSyncFileWriter.h>
#include <util/HeliosException.h>

#include <string>
#include <sstream>
#include <memory>

namespace helios { namespace filems{

using std::string;
using std::stringstream;
using std::shared_ptr;
using std::make_shared;

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
 * @brief SyncFileWriter Factory class. Used to create the appropriate writers
 * based on input flags
 */
class SyncFileWriterFactory
{

public:
/**
 * @brief Synchronous file writer factory
 * @see SyncFileWriterFactory::WriterType
 * @see SyncFileWriter::path
 * @param compress Specify is use compressed LAS format (LAZ) or not (pure
 *  LAS)
 * @see LasSyncFileWriter::scaleFactor
 * @see LasSyncFileWriter::offset
 * @see LasSyncFileWriter::minIntensity
 * @see LasSyncFileWriter::deltaIntensity
 */
static shared_ptr<SyncFileWriter> makeWriter(
    WriterType type, const string &path, bool compress = false,
    double scaleFactor = 0.0001, glm::dvec3 offset = glm::dvec3(0, 0, 0),
    double minIntensity = 0.0, double deltaIntensity = 1000000.0
){
    switch (type) {
        case las10Type:
            return make_shared<LasSyncFileWriter>(
                path,                                // Output path
                compress,                            // Zip flag
                scaleFactor,                         // Scale factor
                offset,                              // Offset
                minIntensity,                        // Min intensity
                deltaIntensity                       // Delta intensity
            );
      case las14Type:
            return make_shared<Las14SyncFileWriter>(
                path,                                // Output path
                compress,                            // Zip flag
                scaleFactor,                         // Scale factor
                offset,                              // Offset
                minIntensity,                        // Min intensity
                deltaIntensity                       // Delta intensity
            );
      case zipType:
            return make_shared<ZipSyncFileWriter>(path);
      case simpleType:
            return make_shared<SimpleSyncFileWriter>(path);
  }

      // Handle unexpected type
      stringstream ss;
      ss    << "SyncFileWriterFactory::makeWriter received an unexpected type:"
            << " (" << type << ")" ;
      throw HeliosException(ss.str());
}
};

}}
