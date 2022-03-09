#pragma once

#include <filems/write/comps/LasSyncFileWriter.h>
#include <filems/write/strategies/LasMeasurementWriteStrategy.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>
#include <Measurement.h>

#include <glm/glm.hpp>

#include <memory>
#include <string>

// TODO Rethink : Use this class through Vectorial FMS factory methods

namespace helios{ namespace filems{

using std::make_shared;

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
class Las14VectorialSyncFileMeasurementWriter :
    public LasSyncFileWriter<
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
     *  ( filems::LasSyncFileWriter::writeStrategy )
     * @see filems::LasMeasurementWriteStrategy
     */
    LasMeasurementWriteStrategy lmws;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    explicit Las14VectorialSyncFileMeasurementWriter(
        const std::string &path,
        bool const compress = false,
        double const scaleFactor = 0.0001,
        glm::dvec3 const offset = glm::dvec3(0, 0, 0),
        double const minIntensity = 0.0,
        double const deltaIntensity = 1000000.0,
        bool const createWriter = true
    ) :
        LasSyncFileWriter<
            vector<Measurement> const &,
            glm::dvec3 const &
        >(
            path,
            compress,
            scaleFactor,
            offset,
            minIntensity,
            deltaIntensity,
            createWriter
        ),
        lmws(
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
        )
    {
        // Write strategy
        this->writeStrategy = make_shared<VectorialWriteStrategy<
            Measurement,
            glm::dvec3 const &
        >>(lmws);
    }
};

}}