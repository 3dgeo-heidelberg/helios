#pragma once

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface that must be implemented by any class which supports
 *  write implementations for file writers
 * @tparam WriteArgs Arguments for the write operation
 * @see filems::SyncFileWriter
 */
template<typename... WriteArgs>
class WriteStrategy
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for write strategy
   */
  WriteStrategy() = default;
  virtual ~WriteStrategy() = default;

  // ***  WRITE STRATEGY INTERFACE *** //
  // ********************************* //
  /**
   * @brief Abstract write function. Must be overridden by
   * children classes.
   */
  virtual void write(WriteArgs... writeArgs) = 0;
};

}
}
