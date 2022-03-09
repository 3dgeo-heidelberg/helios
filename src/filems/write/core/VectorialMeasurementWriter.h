#pragma once

#include <util/HeliosException.h>
#include <filems/write/core/BaseMeasurementWriter.h>
#include <filems/factory/SyncFileMeasurementWriterFactory.h>

#include <string>
#include <memory>

namespace helios { namespace filems{

using std::string;
using std::shared_ptr;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Common implementation for any measurement writer
 */
class VectorialMeasurementWriter :
    public BaseMeasurementWriter<vector<Measurement> const&, glm::dvec3 const&>
{
protected:
    // ***  USING  *** //
    // *************** //
    using BaseMeasurementWriter<
        vector<Measurement> const&, glm::dvec3 const&
    >::scanner;
    using BaseMeasurementWriter<
        vector<Measurement> const&, glm::dvec3 const&
    >::shift;
    using BaseMeasurementWriter<
        vector<Measurement> const&, glm::dvec3 const&
    >::writers;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for vectorial measurement writer
     */
    VectorialMeasurementWriter() :
        BaseMeasurementWriter<vector<Measurement> const&, glm::dvec3 const&>()
    {}
    virtual ~VectorialMeasurementWriter() = default;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @brief Write a vector of measurements
     * @param measurements Vector of measurements to be written
     */
    void writeMeasurements(vector<Measurement> const &measurements);
    /**
     * @brief Like filems::VectorialMeasurementWriter::writeMeasurements but
     *  faster because there is no validation
     * @see filems::VectorialMeasurementWriter::writeMeasurements
     */
    inline void writeMeasurementsUnsafe(
        vector<Measurement> const &measurements
    ) const{sfw->write(measurements, shift);}
    /**
     * @brief Make a vectorial measurement SyncFileWriter
     * @see BaseMeasurementWriter::makeWriter
     */
    shared_ptr<
        SyncFileWriter<vector<Measurement> const &, glm::dvec3 const &>
    > makeWriter(
        WriterType const &type,
        string const &path,
        bool const zipOutput,
        double const lasScale,
        glm::dvec3 shift,
        double const minIntensity,
        double const deltaIntensity
    ) const override{
        return SyncFileMeasurementWriterFactory::makeVectorialWriter(
            type,                   // Writer type
            path,                   // Output path
            isZipOutput(),          // Zip flag
            getLasScale(),          // Scale factor
            shift,                  // Offset
            0.0,                    // Min intensity
            1000000.0               // Delta intensity
        );
    }
};

}}