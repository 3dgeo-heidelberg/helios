#pragma once

#include <SimpleSyncFileWriter.h>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/archive/binary_oarchive.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief ZipSyncFileWriter implementation fro zipped text output format
 */
class ZipSyncFileWriter : public SimpleSyncFileWriter {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Compressed output stream
     */
    boost::iostreams::filtering_ostream compressedOut;
    /**
     * @brief ZLib compression parameters
     */
    boost::iostreams::zlib_params zp;
    /**
     * @brief Binary output archive
     */
    std::unique_ptr<boost::archive::binary_oarchive> oa;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a ZipSyncFileWriter
     * @param compressionMode Compression mode.
     * Use boost::iostreams::zlib::best_speed to reduce execution time at most.
     * Use boost::iostreams::zlib::best_compression to reduce size at most.
     */
    explicit ZipSyncFileWriter(
        const std::string &path,
        int compressionMode = boost::iostreams::zlib::best_compression
    ) :
        SimpleSyncFileWriter(path, std::ios_base::out | std::ios_base::binary)
    {
        zp = boost::iostreams::zlib_params(compressionMode);
        compressedOut.push(boost::iostreams::zlib_compressor(zp));
        compressedOut.push(ofs);
        oa = std::unique_ptr<boost::archive::binary_oarchive>(
            new boost::archive::binary_oarchive(compressedOut)
        );
    }

    // ***  W R I T E  *** //
    // ******************* //
    /**
     * @brief Write measurement to compressed file
     * @see SimpleSyncFileWriter::_write(Measurement const &, glm::dvec3 const &)
     */
    void _write(Measurement const &m, glm::dvec3 const & shift) override{
        (*oa) << measurementToString(m, shift);
    }

    /**
     * @brief Write fullwave to compressed file
     * @see SimpleSyncFileWriter::_write(
     *  std::vector<double> const &,
     *  int, double, double, glm::dvec3, glm::dvec3, long
     * )
     */
    void _write(
        std::vector<double> const &fullwave,
        int fullwaveIndex,
        double minTime,
        double maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long gpsTime
    ) override {
        (*oa) << fullwaveToString(
            fullwave,
            fullwaveIndex,
            minTime,
            maxTime,
            beamOrigin,
            beamDir,
            gpsTime
        );
    }

    /**
     * @brief Write trajectory to compressed file
     * @see SimpleSyncFileWriter::_write(
     *  long,
     *  double, double, double,
     *  double, double, double
     * )
     */
    void _write(const Trajectory &t) override {
        (*oa) << trajectoryToString(t);
    }
};