#pragma once

#include <util/logger/logging.hpp>

#include <sstream>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class defining common behavior for all synchronous file
 *  writers
 * @tparam WriteArgs Arguments for the write operation
 */
template<typename... WriteArgs>
class SyncFileWriter
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for synchronous file writer
   */
  SyncFileWriter() = default;
  virtual ~SyncFileWriter() {}

  // ***  W R I T E  *** //
  // ******************* //
  /**
   * @brief Handle synchronous write operations
   */
  virtual void write(WriteArgs... writeArgs) = 0;

  // ***  F I N I S H  *** //
  // ********************* //
  /**
   * @brief Finish the writing so all writing operations are performed and
   * all buffers are closed
   */
  virtual void finish() {}

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain the path to the file corresponding to the idx-th writing
   *  stream
   * @param idx The index of the writing stream which path must be obtained
   * @return Path to the file corresponding to the idx-th writing stream
   */
  virtual std::string getPath(size_t const idx) const = 0;
  /**
   * @brief Non index version of the SyncFileWriter::getPath(size_t const)
   *  function.
   * @see SyncFileWriter::getPath(size_t const)
   */
  inline std::string getPath() { return getPath(0); }
};

}
}
