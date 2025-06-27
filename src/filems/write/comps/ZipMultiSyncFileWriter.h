#pragma once

#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <filems/write/comps/SimpleMultiSyncFileWriter.h>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract child of SimpleMultiSyncFileWriter which provides support
 *  for zipped output
 * @see filems::SimpleMultiSyncFileWriter
 */
template<typename... WriteArgs>
class ZipMultiSyncFileWriter : public SimpleMultiSyncFileWriter<WriteArgs...>
{
protected:
  using SimpleMultiSyncFileWriter<WriteArgs...>::ofs;
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Compressed output streams
   */
  std::vector<boost::iostreams::filtering_ostream> compressedOut;
  /**
   * @brief ZLib compression parameters for each stream
   */
  std::vector<boost::iostreams::zlib_params> zp;
  /**
   * @brief Binary output archive for each steam
   */
  std::vector<std::unique_ptr<boost::archive::binary_oarchive>> oa;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a ZipMultiSyncFileWriter
   * @param compressionMode Compression mode
   * Use boost::iostreams::zlib::best_speed to reduce execution time at most.
   * Use boost::iostreams::zlib::best_compression to reduce size at most.
   */
  explicit ZipMultiSyncFileWriter(
    std::vector<std::string> const& path,
    int compressionMode = boost::iostreams::zlib::best_compression)
    : SimpleMultiSyncFileWriter<WriteArgs...>(path,
                                              std::ios_base::out |
                                                std::ios_base::binary)
    , compressedOut(path.size())
  {
    // Build and prepare each stream
    std::size_t const nStreams = path.size();
    for (std::size_t i = 0; i < nStreams; ++i) {
      zp.emplace_back(boost::iostreams::zlib_params(compressionMode));
      compressedOut[i].push(boost::iostreams::zlib_compressor(zp[i]));
      compressedOut[i].push(ofs[i]);
      oa.emplace_back(std::unique_ptr<boost::archive::binary_oarchive>(
        new boost::archive::binary_oarchive(compressedOut[i])));
    }
  }
  virtual ~ZipMultiSyncFileWriter() = default;

  // ***  F I N I S H  *** //
  // ********************* //
  /**
   * @brief ZipSyncFileWriter finishes the binary output archive instead of
   *  calling its parent finish method. This is necessary to prevent
   *  malfunctions coming from interaction between output archive closing
   *  and output stream closing
   * @see SimpleSyncFileWriter::finish
   */
  void finish() override
  {
    size_t const nStreams = this->path.size();
    for (size_t i = 0; i < nStreams; ++i) {
      oa[i] = nullptr;
    }
    // SimpleMultiSyncFileWriter<WriteArgs...>::finish();  // Must not call
  }
};

}
}
