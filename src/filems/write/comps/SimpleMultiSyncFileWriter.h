#pragma once

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract specialization of MultiSyncFileWriter to write output
 *  to many files. Each writing operation will be written to one file or
 *  another depending on the WriteArgs
 * @see filems::MultiSyncFileWriter
 * @see filems::SyncFileWriter
 */
template<typename... WriteArgs>
class SimpleMultiSyncFileWriter : public MultiSyncFileWriter<WriteArgs...>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Output file streams to be used by the simple multi synchronous
   *  file writer
   */
  std::vector<std::ofstream> ofs;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous multi-file writer constructor
   * @param path Paths to the output files
   * @param om Open mode for the files (append by default)
   */
  explicit SimpleMultiSyncFileWriter(
    std::vector<std::string> const& path,
    std::ios_base::openmode om = std::ios_base::app)
    : MultiSyncFileWriter<WriteArgs...>(path)
    , ofs(path.size())
  {
    // For each file ...
    size_t const numFiles = path.size();
    for (size_t i = 0; i < numFiles; ++i) {
      // Open it for writing ...
      std::ofstream& ofsi = ofs[i];
      ofsi.open(path[i], om);
      ofsi.exceptions(std::ios_base::eofbit | std::ios_base::failbit |
                      std::ios_base::badbit);
    }
  }
  virtual ~SimpleMultiSyncFileWriter() { finish(); }

  // ***  F I N I S H  *** //
  // ********************* //
  /**
   * @brief SimpleMultiSyncFileWriter finish method assures that any output
   *  file stream that remains open is closed.
   */
  void finish() override
  {
    for (std::ofstream& ofs : ofs) {
      if (ofs.is_open())
        ofs.close();
    }
  }
};

}
}
