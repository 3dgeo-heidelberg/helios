#pragma once

#include <helios/filems/read/comps/SimpleFileReader.h>
#include <helios/filems/read/strategies/BinaryReadingStrategy.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class for byte by byte reading of binary files
 * @see filems::SimpleFileReader
 */
class BinaryFileReader : public SimpleFileReader<int>
{
protected:
  // ***  USING  *** //
  // *************** //
  using SimpleFileReader<int>::readingStrategy;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for binary file reader
   * @see filems::SimpleFileReader::SimpleFileReader
   */
  BinaryFileReader(std::string const& path,
                   std::ios_base::openmode openMode = std::ios_base::in)
    : SimpleFileReader<int>(path, std::ios_base::in)
  {
    makeStrategy();
  }
  virtual ~BinaryFileReader() = default;

protected:
  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Build a binary reading strategy for the binary file reader
   * @see FileReader::makeStrategy
   */
  void makeStrategy() override
  {
    readingStrategy = std::make_shared<BinaryReadingStrategy>(ifs);
  }
};

}
}
