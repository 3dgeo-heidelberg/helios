#pragma once

#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/strategies/DirectMeasurementWriteStrategy.h>

#include <memory>

namespace helios { namespace filems{

using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write
 *  measurements directly to a file.
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectMeasurementWriteStrategy
 * @see Measurement
 */
class SimpleSyncFileMeasurementWriter :
    public SimpleSyncFileWriter<Measurement const &, glm::dvec3 const>
{

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simple synchronous file measurement writer constructor
     * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
     */
    explicit SimpleSyncFileMeasurementWriter(
        const std::string& path,
        std::ios_base::openmode om = std::ios_base::app
    ) :
        SimpleSyncFileWriter<Measurement const&, glm::dvec3 const>(path, om)
    {
        this->writeStrategy = make_shared<DirectMeasurementWriteStrategy>(
            this->ofs
        );
    }
    virtual ~SimpleSyncFileMeasurementWriter() = default;
};

}}
