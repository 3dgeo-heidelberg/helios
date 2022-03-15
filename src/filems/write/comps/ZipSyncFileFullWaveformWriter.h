#pragma once

#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/strategies/ZipFullWaveformWriteStrategy.h>

#include <memory>
#include <vector>

namespace helios{ namespace filems{

using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipSyncFileWriter to write compressed
 *  full waveform data to a file
 *
 * @see filems::ZipSyncFileWriter
 * @see filems::ZipFullWaveformWriteStrategy
 */
class ZipSyncFileFullWaveformWriter :
    public ZipSyncFileWriter<
        std::vector<double> const &,
        int const,
        double const,
        double const,
        glm::dvec3 const&,
        glm::dvec3 const&,
        long const
    >
{

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simple synchronous zipped full waveform writer constructor
     * @see filems:ZipSyncFileWriter::ZipSyncFileWriter
     */
    explicit ZipSyncFileFullWaveformWriter(
        string const &path,
        int compressionMode = boost::iostreams::zlib::best_compression
    ) :
        ZipSyncFileWriter<
            std::vector<double> const&,
            int const,
            double const,
            double const,
            glm::dvec3 const&,
            glm::dvec3 const&,
            long const
        >(path, compressionMode)
    {
        this->writeStrategy = make_shared<ZipFullWaveformWriteStrategy>(
            this->ofs,
            *(this->oa)
        );
    }
    virtual ~ZipSyncFileFullWaveformWriter() = default;
};

}}