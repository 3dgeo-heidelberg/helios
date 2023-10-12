#pragma once

#include <filems/write/comps/ZipSyncFileWriter.h>
#include <filems/write/strategies/ZipPulseWriteStrategy.h>
#include <filems/write/strategies/VectorialWriteStrategy.h>

#include <memory>
#include <vector>

namespace helios { namespace filems{

using std::make_shared;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipSyncFileWriter to write a vector of
 *  pulse records to a zip file.
 *
 * @see filems::ZipSyncFileWriter
 * @see filems::ZipPulseWriteStrategy
 * @see PulseRecord
 * @see filems::ZipSyncFilePulseWriter
 */
class ZipVectorialSyncFilePulseWriter :
    public ZipSyncFileWriter<vector<PulseRecord> const &> {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The pulse write strategy that is wrapped by the main write
     *  strategy in a vectorial fashion
     *  ( filems::ZipSyncFileWriter::writeStrategy )
     * @see filems::ZipPulseWriteStrategy
     */
    ZipPulseWriteStrategy zpws;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief ZIP synchronous file pulse vector writer constructor
     * @see filems::ZipSyncFileWriter::ZipSyncFileWriter
     */
    explicit ZipVectorialSyncFilePulseWriter(
        const string &path,
        int compressionMode = boost::iostreams::zlib::best_compression
    ) :
        ZipSyncFileWriter<vector<PulseRecord> const &>(path, compressionMode),
        zpws(this->ofs, *(this->oa))
    {
        this->writeStrategy = make_shared<VectorialWriteStrategy<
            PulseRecord
        >>(zpws);
    }
    virtual ~ZipVectorialSyncFilePulseWriter() = default;
};

}}