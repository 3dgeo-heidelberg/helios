#pragma once

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class declaring the methods that any file writer must
 *  implement ( FileWriter::write ) and allowing for an optional redefinition
 *  of the method that handles the end of life of the writer
 *  ( FileWriter::finish )
 */
template<typename... WriteArgs>
class FileWriter
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the FileWriter class
   */
  FileWriter() = default;
  virtual ~FileWriter() = default;

  // ***  W R I T E  *** //
  // ******************* //
  /**
   * @brief The write method. Any concrete file writer must override this
   *  method and provide a valid implementation.
   */
  virtual void write(WriteArgs... writeArgs) = 0;

  // ***  F I N I S H  *** //
  // ********************* //
  /**
   * @see This function can be overriden by any implementation that needs
   *  to do something before the writer is disabled or deleted (e.g.,
   *  flushing output streams)
   */
  virtual void finish() {}
};
}
}
