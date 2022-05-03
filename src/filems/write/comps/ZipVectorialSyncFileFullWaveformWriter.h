#pragma once

#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/strategies/ZipFullWaveformWriteStrategy.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>
#include <scanner/detector/FullWaveform.h>

#include <memory>
#include <vector>

namespace helios { namespace filems{

using std::make_shared;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipSyncFileWriter to write a vector of
 *  full waveform to a zip file.
 *
 * @see filems::ZipSyncFileWriter
 * @see filems::ZipFullWaveformWriteStrategy
 * @see FullWaveform
 * @see filems::ZipSyncFileFullWaveformWriter
 */
class ZipVectorialSyncFileFullWaveformWriter :
    public ZipSyncFileWriter<vector<FullWaveform> const &>
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The full waveform write strategy that is wrapped by the main
     *  write strategy in a vectorial fashion
     *  ( filems::ZipSyncFileWriter::writeStrategy )
     * @see filems::ZipFullWaveformWriteStrategy
     */
    ZipFullWaveformWriteStrategy zfwws;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief ZIP synchronous file full waveform vector writer constructor
     * @see filems::ZipSyncFileWriter::ZipSyncFileWriter
     */
    explicit ZipVectorialSyncFileFullWaveformWriter(
        const string &path,
        int compressionMode = boost::iostreams::zlib::best_compression
    ) :
        ZipSyncFileWriter<vector<FullWaveform> const&>(path, compressionMode),
        zfwws(this->ofs, *(this->oa))
    {
        this->writeStrategy = make_shared<VectorialWriteStrategy<
            FullWaveform
        >>(zfwws);
    }
    virtual ~ZipVectorialSyncFileFullWaveformWriter() = default;
};

}}