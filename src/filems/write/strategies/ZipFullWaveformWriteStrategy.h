#pragma once

#include <filems/write/strategies/DirectFullWaveformWriteStrategy.h>

#include <boost/archive/binary_oarchive.hpp>

namespace helios { namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Like DirectFullWaveformWriteStrategy but zipping the output
 * @see filems::DirectFullWaveformWriteStrategy
 */
class ZipFullWaveformWriteStrategy : public DirectFullWaveformWriteStrategy{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The zipping output stream to do the writing. It must be
     *  associated to the file output stream of the parent
     *  DirectFullWaveformWriteStrategy
     */
    boost::archive::binary_oarchive &oa;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for zip full waveform write strategy
     * @see ZipFullWaveformWriteStrategy::oa
     * @see DirectFullWaveformWriteStrategy::DirectFullWaveformWriteStrategy
     */
    ZipFullWaveformWriteStrategy(
        std::ofstream &ofs,
        boost::archive::binary_oarchive &oa
    ) :
        DirectFullWaveformWriteStrategy(ofs),
        oa(oa)
    {}
    ~ZipFullWaveformWriteStrategy() override = default;

    // ***  WRITE STRATEGY INTERFACE *** //
    // ********************************* //
    /**
     * @brief Write full waveform to compressed file
     * @see DirectFullWaveformWriteStrategy::write
     */
    void write(FullWaveform const &fullWaveform) override{
        oa << fullWaveformToString(fullWaveform);
    }
};

}}