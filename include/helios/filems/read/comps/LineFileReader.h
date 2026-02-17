#pragma once

#include <helios/filems/read/comps/SimpleFileReader.h>
#include <helios/filems/read/strategies/LineReadingStrategy.h>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class supporting line by line reading of text files
 * @see filems::SimpleFileReader
 * @see filems::LineReadingStrategy
 */
class LineFileReader : public SimpleFileReader<std::string>
{
protected:
  // ***  USING  *** //
  // *************** //
  using SimpleFileReader<std::string>::readingStrategy;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The maximum number of characters that a line is expected to have.
   *
   * If a file having lines of more than maxCharsPerLine characters is read,
   *  then problems might arise
   * @see filems::LineReadingStrategy::maxCharsPerLine
   */
  long maxCharsPerLine;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for line file reader
   * @see filems::SimpleFileReader::SimpleFileReader
   */
  LineFileReader(std::string const& path,
                 std::ios_base::openmode openMode = std::ios_base::in,
                 long const maxCharsPerLine = 8192,
                 bool const constructStrategy = true)
    : SimpleFileReader<std::string>(path, openMode)
    , maxCharsPerLine(maxCharsPerLine)
  {
    if (constructStrategy)
      makeStrategy();
  }
  virtual ~LineFileReader() = default;

protected:
  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Build a line reading strategy for the line file reader
   * @see FileReader::makeStrategy
   */
  void makeStrategy() override
  {
    readingStrategy =
      std::make_shared<LineReadingStrategy>(ifs, maxCharsPerLine);
  }

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the maximum number of characters per line
   * @return Maximum number of characters per line
   * @see filems::LineFileReader::maxCharsPerLine
   * @see filems::LineFileReader::setMaxCharsPerLine()
   */
  virtual inline long getMaxCharsPerLine() { return maxCharsPerLine; }
  /**
   * @brief Set the maximum number of characters per line and update strategy
   *  accordingly
   * @param maxCharsPerLine New maximum number of characters per line
   * @see filems::LineFileReader::maxCharsPerLine
   * @see filems::LineFileReader::getMaxCharsPerLine()
   */
  virtual void setMaxCharsPerLine(long const maxCharsPerLine)
  {
    this->maxCharsPerLine = maxCharsPerLine;
    makeStrategy();
  }
};

}
}
