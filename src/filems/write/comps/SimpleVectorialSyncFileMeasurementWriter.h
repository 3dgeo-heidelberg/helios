#pragma once

#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/strategies/DirectMeasurementWriteStrategy.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>

#include <memory>
#include <vector>


namespace helios { namespace filems{

using std::make_shared;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing SimpleSyncFileWriter to write a
 *  vector of measurements directly to a file.
 *
 * @see filems::SimpleSyncFileWriter
 * @see filems::DirectMeasurementWriteStrategy
 * @see Measurement
 * @see SimpleSyncFileMeasurementWriter
 */
class SimpleVectorialSyncFileMeasurementWriter :
    public SimpleSyncFileWriter<
        vector<Measurement> const &,
        glm::dvec3 const &
    >
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The measurement write strategy that is wrapped by the main write
     *  strategy in a vectorial fashion
     *  ( filems::SimpleSyncFileWriter::writeStrategy )
     * @see filems::DirectMeasurementWriteStrategy
     */
    DirectMeasurementWriteStrategy dmws;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simple synchronous file measurement vector writer constructor
     * @see filems::SimpleSyncFileWriter::SimpleSyncFileWriter
     */
    explicit SimpleVectorialSyncFileMeasurementWriter(
        const std::string& path,
        std::ios_base::openmode om = std::ios_base::app
    ) :
        SimpleSyncFileWriter<
            vector<Measurement> const&,
            glm::dvec3 const&
        >(path, om),
        dmws(this->ofs)
    {
        this->writeStrategy = make_shared<VectorialWriteStrategy<
            Measurement,
            glm::dvec3 const &
        >>(dmws);
    }
    virtual ~SimpleVectorialSyncFileMeasurementWriter() = default;
};

}}
