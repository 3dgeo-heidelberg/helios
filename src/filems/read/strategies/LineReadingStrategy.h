#pragma once

#include <filems/read/exceptions/EndOfReadingException.h>
#include <filems/read/strategies/SimpleReadingStrategy.h>

#include <fstream>
#include <string>

namespace helios {
namespace filems {

using std::fstream;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class defining the strategy to read line by line from a file input
 *  stream
 * @see filems::SimpleReadingStrategy
 */
class LineReadingStrategy : public SimpleReadingStrategy<string>
{
protected:
  // ***  USING  *** //
  // *************** //
  using SimpleReadingStrategy<string>::ifs;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The maximum number of characters that a line is expected to have.
   *
   * If a file having lines of more than maxCharsPerLine characters is read,
   *  then problems might arise
   * @see filems::LineFileReader::maxCharsPerLine
   */
  long const& maxCharsPerLine;
  /**
   * @brief Buffer where the read line is stored
   */
  char* buffer = nullptr;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for line reading strategy
   * @see filems::SimpleReadingStrategy::SimpleReadingStrategy
   */
  LineReadingStrategy(ifstream& ifs, long const& maxCharsPerLine)
    : SimpleReadingStrategy<string>(ifs)
    , maxCharsPerLine(maxCharsPerLine)
  {
    buffer = new char[maxCharsPerLine];
  }
  virtual ~LineReadingStrategy() { delete[] buffer; }

  // ***  READING STRATEGY METHODS  *** //
  // ********************************** //
  /**
   * @brief Read line from text file
   * @return Either what has been read from input file or throw an exception
   *  if end of file was reached ( filems::EndOfReadingException )
   * @see helios::filems::EndOfReadingException
   */
  string read() override
  {
    if (ifs.getline(buffer, maxCharsPerLine))
      return string(buffer);
    throw EndOfReadingException();
  }
};

}
}
