#pragma once

#include <filems/write/strategies/WriteStrategy.h>
#include <scanner/Measurement.h>

#include <glm/glm.hpp>

#include <fstream>
#include <sstream>

namespace helios { namespace filems{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to directly write
 *  measurements to a file.
 * @see filems::WriteStrategy
 * @see filems::SimpleSyncFileMeasurementWriter
 * @see filems::VectorialMeasurementWriteStrategy
 */
class DirectMeasurementWriteStrategy :
    public WriteStrategy<Measurement const &, glm::dvec3 const &>
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The output file stream to do the writing
     */
    std::ofstream &ofs;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for direct measurement write strategy
     * @see DirectMeasurementWriteStrategy::ofs
     */
    DirectMeasurementWriteStrategy(std::ofstream &ofs) : ofs(ofs) {}
    virtual ~DirectMeasurementWriteStrategy() = default;

    // ***  WRITE STRATEGY INTERFACE *** //
    // ********************************* //
    /**
     * @brief Write measurement to file
     * @param m Measurement to be written
     * @param shift Shift for the measurement position
     * @see Measurement
     */
    void write(Measurement const &m, glm::dvec3 const & shift) override{
        ofs << measurementToString(m, shift);
    }

protected:
    // ***  UTILS  *** //
    // *************** //
    /**
     * @brief Build a string from measurement data
     * @param m Measurement data itself
     * @param shift Shift for the measurement coordinates
     * @return String with measurement data
     */
    virtual std::string measurementToString(
        Measurement const &m,
        glm::dvec3 const & shift
    ){
        glm::dvec3 shifted = m.position + shift;
        std::stringstream ss;
        ss << std::setprecision(4) << std::fixed;
        ss  << shifted.x << " "
            << shifted.y << " "
            << shifted.z << " "
            << m.intensity << " "
            << m.echo_width << " "
            << m.returnNumber << " "
            << m.pulseReturnNumber << " "
            << m.fullwaveIndex << " "
            << m.hitObjectId << " "
            << m.classification << " "
            << std::setprecision(4) << std::fixed
            << ((double)m.gpsTime) / 1000.0 << std::endl;
        return ss.str();
    }
};

    }}