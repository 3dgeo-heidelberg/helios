#pragma once

#include <filems/write/strategies/DirectMeasurementWriteStrategy.h>

#include <boost/archive/binary_oarchive.hpp>

namespace helios { namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Like DirectMeasurementWriteStrategy but zipping the output
 * @see filems::DirectMeasurementWriteStrategy
 */
class ZipMeasurementWriteStrategy : public DirectMeasurementWriteStrategy{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The zipping output stream to do the writing. It must be
     *  associated to the file output stream of the parent
     *  DirectMeasurementWriteStrategy
     */
    boost::archive::binary_oarchive &oa;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for zip measurement write strategy
     * @see ZipMeasurementWriteStrategy::oa
     * @see DirectMeasurementWriteStrategy::DirectMeasurementWriteStrategy
     */
    ZipMeasurementWriteStrategy(
        std::ofstream &ofs,
        boost::archive::binary_oarchive &oa
    ) :
        DirectMeasurementWriteStrategy(ofs),
        oa(oa)
    {}
    ~ZipMeasurementWriteStrategy() override = default;

    // ***  WRITE STRATEGY INTERFACE *** //
    // ********************************* //
    /**
     * @brief Write measurement to compressed file
     * @see DirectMeasurementWriteStrategy::write
     */
    void write(Measurement const &m, glm::dvec3 const & shift) override{
        oa << measurementToString(m, shift);
    }
};

}}