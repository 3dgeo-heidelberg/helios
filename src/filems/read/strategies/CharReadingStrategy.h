#pragma once

#include <filems/read/strategies/SimpleReadingStrategy.h>

#include <fstream>

namespace helios {
namespace filems {

using std::fstream;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class defining the strategy to read char by char from a file input
 *  stream
 * @see filems::SimpleReadingStrategy
 */
class CharReadingStrategy : public SimpleReadingStrategy<char>
{
protected:
  // ***  USING  *** //
  // *************** //
  using SimpleReadingStrategy<char>::ifs;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for char reading strategy
   * @see filems::SimpleReadingStrategy::SimpleReadingStrategy
   */
  CharReadingStrategy(ifstream& ifs)
    : SimpleReadingStrategy<char>(ifs)
  {
  }
  virtual ~CharReadingStrategy() = default;

  // ***  READING STRATEGY METHODS  *** //
  // ********************************** //
  /**
   * @brief Read character from text file
   * @return Either what has been read from input file or EOF if end of file
   *  was reached
   */
  char read() override { return ifs.get(); };
};

}
}
