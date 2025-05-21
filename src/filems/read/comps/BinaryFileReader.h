#pragma once

#include <filems/read/comps/SimpleFileReader.h>
#include <filems/read/strategies/BinaryReadingStrategy.h>

#include <memory>

namespace helios {
namespace filems {

using std::ios_base;
using std::make_shared;

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
  BinaryFileReader(string const& path,
                   ios_base::openmode openMode = ios_base::in)
    : SimpleFileReader<int>(path, in)
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
    readingStrategy = make_shared<BinaryReadingStrategy>(ifs);
  }
};

}
}
