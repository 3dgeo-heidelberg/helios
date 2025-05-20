#pragma once

#include <filems/read/strategies/ReadingStrategy.h>

#include <fstream>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class defining the strategy to read from a simple file input
 *  stream
 */
template<typename ReadArg>
class SimpleReadingStrategy : public ReadingStrategy<ReadArg>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Reference to the input file stream to read from
   */
  std::ifstream& ifs;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for simple reading strategy
   */
  SimpleReadingStrategy(std::ifstream& ifs)
    : ReadingStrategy<ReadArg>()
    , ifs(ifs)
  {
  }
  virtual ~SimpleReadingStrategy() = default;
};

}
}
