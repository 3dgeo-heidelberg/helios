#pragma once

#include <filems/read/exceptions/EndOfReadingException.h>
#include <filems/read/strategies/SimpleReadingStrategy.h>

#include <fstream>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class defining the strategy to read line by line from a file input
 *  stream
 * @see filems::SimpleReadingStrategy
 */
class LineReadingStrategy : public SimpleReadingStrategy<std::string>
{
protected:
  // ***  USING  *** //
  // *************** //
  using SimpleReadingStrategy<std::string>::ifs;

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
  LineReadingStrategy(std::ifstream& ifs, long const& maxCharsPerLine)
    : SimpleReadingStrategy<std::string>(ifs)
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
  std::string read() override
  {
    if (ifs.getline(buffer, maxCharsPerLine))
      return std::string(buffer);
    throw EndOfReadingException();
  }
};

}
}
