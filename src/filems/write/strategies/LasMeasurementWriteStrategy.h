#pragma once

#include <filems/write/strategies/WriteStrategy.h>
#include <scanner/Measurement.h>

#include <laswriter.hpp>
#include <glm/glm.hpp>


namespace helios { namespace filems{


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to directly write
 *  measurements to a file with LAS format
 * @see filems::WriteStrategy
 * @see filems::LasSyncFileMeasurementWriter
 */
class LasMeasurementWriteStrategy :
    public WriteStrategy<Measurement const &, glm::dvec3 const &>
{
public:
    // ***  CONSTANTS  *** //
    // ******************* //
    /**
     * @brief Classification mask constant for LAS format
     */
    static const U8 CLASSIFICATION_MASK = 31;
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The LASwriter to write points to LAS file.
     * @see filems::LasSyncFileMeasurementWriter::lw
     */
    LASwriter &lw;
    /**
     * @brief The LASpoint configured to build points for the desired LAS
     *  output format.
     * @see filems::LasSyncFileMeasurementWriter::lp
     */
    LASpoint &lp;
    /**
     * @see filems::LasSyncFileMeasurementWriter::scaleFactorInverse
     */
    double const &scaleFactorInverse;
    /**
     * @see filems::LasSyncFileMeasurementWriter::offset
     */
    glm::dvec3 const &offset;
    /**
     * @see filems::LasSyncFileMeasurementWriter::minIntensity
     */
    double const &minIntensity;
    /**
     * @see filems::LasSyncFileMeasurementWriter::maxIntensity
     */
    double const &maxIntensity;
    /**
     * @see filems::LasSyncFileMeasurementWriter::intensityCoefficient
     */
    double const &intensityCoefficient;
    /**
     * @see filems::LasSyncFileMeasurementWriter::ewAttrStart
     */
    I32 const &ewAttrStart;
    /**
     * @see filems::LasSyncFileMeasurementWriter::fwiAttrStart
     */
    I32 const &fwiAttrStart;
    /**
     * @see filems::LasSyncFileMeasurementWriter::hoiAttrStart
     */
    I32 const &hoiAttrStart;
    /**
     * @see filems::LasSyncFileMeasurementWriter::ampAttrStart
     */
    I32 const &ampAttrStart;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for LAS measurement write strategy
     * @see LasMeasurementWriteStrategy::lw
     * @see LasMeasurementWriteStrategy::lp
     */
    LasMeasurementWriteStrategy(
        LASwriter &lw,
        LASpoint &lp,
        double const &scaleFactorInverse,
        glm::dvec3 const &offset,
        double const &minIntensity,
        double const &maxIntensity,
        double const &intensityCoefficient,
        I32 const &ewAttrStart,
        I32 const &fwiAttrStart,
        I32 const &hoiAttrStart,
        I32 const &ampAttrStart
    ) :
        lw(lw),
        lp(lp),
        scaleFactorInverse(scaleFactorInverse),
        offset(offset),
        minIntensity(minIntensity),
        maxIntensity(maxIntensity),
        intensityCoefficient(intensityCoefficient),
        ewAttrStart(ewAttrStart),
        fwiAttrStart(fwiAttrStart),
        hoiAttrStart(hoiAttrStart),
        ampAttrStart(ampAttrStart)
    {}
    virtual ~LasMeasurementWriteStrategy() = default;

    // ***  WRITE STRATEGY INTERFACE  *** //
    // ********************************** //
    /**
     * @brief Write measurement to LAS file
     * @param m Measurement to be written
     * @param shift Shift for the measurement position
     * @see Measurement
     */
    void write(Measurement const &m, glm::dvec3 const &shift) override {
        measurementToPoint(m, shift);
        lw.write_point(&lp);
        lw.update_inventory(&lp);
    }

protected:
    // ***  UTILS  *** //
    // **************** //
    /**
     * @brief Build a LAS point from measurement data
     * @param m Measurement data itself
     * @param shift Shift for the measurement coordinates
     */
     virtual void measurementToPoint(
         Measurement const &m,
         glm::dvec3 const &shift
     ){
        // Compute shifted position (new impl. shift .las as .xyz)
        //glm::dvec3 shifted = m.position + shift - offset; // Old impl.
        glm::dvec3 shifted = m.position + shift; // New impl.

        // Populate LAS point (base attributes)
        lp.set_X(I32(shifted.x * scaleFactorInverse));
        lp.set_Y(I32(shifted.y * scaleFactorInverse));
        lp.set_Z(I32(shifted.z * scaleFactorInverse));
        double intensityf = m.intensity;
        if(intensityf < minIntensity) intensityf = minIntensity;
        if(intensityf > maxIntensity) intensityf = maxIntensity;
        U16 intensity = U16(
            intensityCoefficient * (intensityf - minIntensity)
        );
        lp.set_intensity(intensity);

        lp.set_return_number(U8(m.returnNumber));
        lp.set_extended_return_number(U8(m.returnNumber));

        lp.set_number_of_returns(U8(m.pulseReturnNumber));
        lp.set_extended_number_of_returns(U8(m.pulseReturnNumber));

        lp.set_classification(U8(m.classification) & CLASSIFICATION_MASK);
        lp.set_extended_classification(U8(m.classification) & CLASSIFICATION_MASK);

        lp.set_gps_time(F64((m.gpsTime)/1000000000.0));

        // Populate LAS point (extra bytes attributes)
        lp.set_attribute(ewAttrStart, F64(m.echo_width));
        lp.set_attribute(fwiAttrStart, I32(m.fullwaveIndex));
        lp.set_attribute(hoiAttrStart, I32(std::stoi(m.hitObjectId)));
        lp.set_attribute(ampAttrStart, F64(m.intensity));
     }
};


}}
