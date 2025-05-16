#pragma once

#include <filems/read/comps/SimpleFileReader.h>
#include <filems/read/strategies/CharReadingStrategy.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class for char by char reading of files
 * @see filems::SimpleFileReader
 * @see filems::CharReadingStrategy
 */
class CharFileReader : public SimpleFileReader<char>
{
protected:
  // ***  USING  *** //
  // *************** //
  using SimpleFileReader<char>::readingStrategy;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for binary file reader
   * @see filems::SimpleFileReader::SimpleFileReader
   */
  CharFileReader(string const& path,
                 std::ios_base::openmode openMode = std::ios_base::in)
    : SimpleFileReader<char>(path, std::ios_base::in)
  {
    makeStrategy();
  }
  virtual ~CharFileReader() = default;

protected:
  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Build a char reading strategy for the char file reader
   * @see FileReader::makeStrategy
   */
  void makeStrategy() override
  {
    readingStrategy = std::make_shared<CharReadingStrategy>(ifs);
  }
};

}
}
