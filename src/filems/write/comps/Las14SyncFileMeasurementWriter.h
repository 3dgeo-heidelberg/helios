#pragma once

// Includes
#include <string>
#include <filems/write/comps/LasSyncFileMeasurementWriter.h>

namespace helios { namespace filems {

using std::make_shared;

/**
 * @author Miguel Yermo García
 * @version 1.0
 * @brief LasSyncFileWriter implementation for LAS v1.4 format
 */
class Las14SyncFileMeasurementWriter : public LasSyncFileMeasurementWriter
{
public:

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    Las14SyncFileMeasurementWriter() : LasSyncFileMeasurementWriter() {};

    explicit Las14SyncFileMeasurementWriter(
        const std::string &path,
        bool compress = false,
        double scaleFactor = 0.0001,
        glm::dvec3 offset = glm::dvec3(0, 0, 0),
        double minIntensity = 0.0,
        double deltaIntensity = 1000000.0
    ) :
        LasSyncFileMeasurementWriter(
            path,
            compress,
            scaleFactor,
            offset,
            minIntensity,
            deltaIntensity,
            false // Prevent parent from creating LAS writer
        )
    {};

    virtual ~Las14SyncFileMeasurementWriter() = default;

    // ***  CREATE WRITER  *** //
    // *********************** //
    /**
     * @brief Creation of the LasWriter itself, including LASpoint
     * initialization but using LAS14 version instead of LAS10
     * @param path Path where the file will be save
     * @param compress Flag to activate/deactivate compression (las/laz format)
     * @see LasSyncFileWriter::createLasWriter
     */
    void createLasWriter(const std::string & path, bool const compress)
    {
        // Craft header and point format for LAS_14 version
        lws.craft14();

        // Add extra attributes
        lws.addExtraAttributes();

        // Initialize LASpoint
        lws.initLASPoint();

        // Create writer from specification
        lw = lws.makeWriter(path, compress);
    }


};

}}
