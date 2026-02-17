#pragma once

#include <helios/filems/write/comps/SingleSyncFileWriter.h>
#include <helios/maths/MathConverter.h>

#include <fstream>
#include <iomanip>
#include <iterator>
#include <ostream>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract specialization of SingleSyncFileWriter to write output
 *  directly to a file
 * @see filems::SingleSyncFileWriter
 * @see filems::SyncFileWriter
 */
template<typename... WriteArgs>
class SimpleSyncFileWriter : public SingleSyncFileWriter<WriteArgs...>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Output file stream to be used by the simple synchronous file
   *  writer
   */
  std::ofstream ofs;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Simple synchronous file writer constructor
   * @param path Path to the output file
   * @param om Open mode for the file (append by default)
   */
  explicit SimpleSyncFileWriter(const std::string& path,
                                std::ios_base::openmode om = std::ios_base::app)
    : SingleSyncFileWriter<WriteArgs...>(path)
  {
    // Open file for writing ...
    ofs.open(path, om);
    ofs.exceptions(std::ios_base::eofbit | std::ios_base::failbit |
                   std::ios_base::badbit);
  }
  virtual ~SimpleSyncFileWriter() { finish(); }

  // ***  F I N I S H  *** //
  // ********************* //
  /**
   * @brief SimpleSyncFileWriter finish method assures that output file
   *  will be closed if it is open
   */
  void finish() override
  {
    if (ofs.is_open())
      ofs.close();
  }
};

}
}
