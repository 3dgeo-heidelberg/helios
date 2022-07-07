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
    {
        // Craft header and point format
        craft();

        // Add extra attributes
        addExtraAttributes();

        // Create LASWriter
        createLasWriter(path, compress);

        // Write strategy
        this->writeStrategy = make_shared<LasMeasurementWriteStrategy>(
            *lw,
            lp,
            scaleFactorInverse,
            this->offset,
            this->minIntensity,
            maxIntensity,
            intensityCoefficient,
            ewAttrStart,
            fwiAttrStart,
            hoiAttrStart,
            ampAttrStart
        );
    };

    virtual ~Las14SyncFileMeasurementWriter() = default;

    // ***  CRAFTING  *** //
    // ****************** //
    /**
     * @brief Crafting of header of the LAS file for version 1.4
     * @see LasSyncFileMeasurementWriter::craft
     */
    void craft(){
        // Craft version of LasSyncWriter LAS 1.0
        LasSyncFileMeasurementWriter::craft();

        // Update Header to 1.4 specification
        lwHeader.version_minor = U8(4);

        // Update Point Data Format to support new return number / classes
        lwHeader.point_data_format = 6;
        lwHeader.point_data_record_length = 30;

        // Adds the byte difference between LAS 1.4 and LAS 1.0 (350 - 227)
        lwHeader.header_size += 148;

        // Adds the byte difference to the data point offset
        lwHeader.offset_to_point_data += 148;
    }


};

}}
