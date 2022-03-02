#pragma once

#include <filems/write/MeasurementWriter.h>

#include <string>
#include <memory>

namespace helios { namespace filems {

using ::std::string;
using ::std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The facade for FMS writing
 */
class FMSWriteFacade{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The writer for measurements
     */
    shared_ptr<MeasurementWriter> mw = nullptr;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for FMS write facade
     */
    FMSWriteFacade() = default;
    virtual ~FMSWriteFacade() = default;

    // ***  FACADE MEASUREMENT WRITE METHODS  *** //
    // ****************************************** //
    /**
     * @brief Obtain the measurement writer of the write facade
     * @return The measurement writer of the write facade
     */
    inline shared_ptr<MeasurementWriter> getMeasurementWriter() const
    {return this->mw;}
    /**
     * @brief Set the measurement writer of the write facade
     * @param mw New measurement writer for the write facade
     */
    inline void setMeasurementWriter(shared_ptr<MeasurementWriter> mw)
    {this->mw = mw;}
    /**
     * @brief Validate the measurement writer of the facade is valid to
     *  support write methods. If it is not valid, an adequate exception
     *  will be thrown.
     * @see FMSWriteFacade::mw
     */
    void validateMeasurementWriter(
        string const &callerName="FMSWriteFacade::validateMeasurementWriter",
        string const &errorMsg="could not access MeasurementWriter"
    ) const;
    /**
     * @see MeasurementWriter::writeMeasurement
     */
    void writeMeasurement(Measurement &m);
    /**
     * @see filems::MeasurementWriter::finish
     */
    void finishMeasurementWriter();
    /**
     * @see filems::MeasurementWriter::getOutputFilePath
     */
    fs::path getMeasurementWriterOutputFilePath();
    /**
     * @see filems::MeasurementWriter::setOutputFilePath
     */
    void setMeasurementWriterOutputFilePath(
        std::string path,
        const bool lastLegInStrip
    );
    /**
     * @see filems::MeasurementWriter::isLasOutput
     */
    bool isMeasurementWriterLasOutput() const;
    /**
     * @see filems::MeasurementWriter::setLasOutput
     */
    void setMeasurementWriterLasOutput(bool const lasOutput);
    /**
     * @see filems::MeasurementWriter::isLas10
     */
    bool isMeasurementWriterLas10() const;
    /**
     * @see filems::MeasurementWriter::setLas10
     */
    void setMeasurementWriterLas10(bool const las10);
    /**
     * @see filems::MeasurementWriter::isZipOutput
     */
    bool isMeasurementWriterZipOutput() const;
    /**
     * @see filems::MeasurementWriter::setZipOutput
     */
    void setMeasurementWriterZipOutput(bool const zipOutput);
    /**
     * @see filems::MeasurementWriter::getLasScale
     */
    double getMeasurementWriterLasScale() const;
    /**
     * @see filems::MeasurementWriter::setLasScale
     */
    void setMeasurementWriterLasScale(double const lasScale);
};

}}