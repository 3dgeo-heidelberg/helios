#pragma once

#include <filems/read/comps/FileReader.h>

#include <fstream>
#include <memory>
#include <sstream>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class defining the fundamental of any file reader that uses
 *  standard file input stream as reading mechanism
 * @tparam ReadArg Type of what is read from file
 * @see filems::FileReader
 */
template<typename ReadArg>
class SimpleFileReader : public FileReader<ReadArg>
{
protected:
  // ***  USING  *** //
  // *************** //
  using FileReader<ReadArg>::setPath;
  using FileReader<ReadArg>::readingStrategy;
  using FileReader<ReadArg>::makeStrategy;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The input file stream to read from
   */
  std::ifstream ifs;
  /**
   * @brief The open mode flags for the input file stream
   * @see filems::SimpleFileReader::ifs
   */
  std::ios_base::openmode openMode;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for simple file reader
   * @see filems::FileReader::FileReader
   * @see filems::SimpleFileReader::ifs
   * @see filems::SimpleFileReader::openMode
   */
  SimpleFileReader(std::string const& path,
                   std::ios_base::openmode openMode = std::ios_base::in)
    : FileReader<ReadArg>(path)
    , ifs(path, openMode)
    , openMode(openMode)
  {
    if (!ifs.is_open()) {
      std::stringstream ss;
      ss << "SimpleFileReader::SimpleFileReader("
         << "string const &, ios__base::openmode"
         << ") failed to open file at path:\n\"" << path << "\"";
      throw std::ios_base::failure(ss.str());
    }
  }
  virtual ~SimpleFileReader() = default;

  // ***  READ METHODS  *** //
  // ********************** //
  /**
   * @brief Read from file simply by applying the reading strategy.
   *  Therefore, there is no concurrency handling mechanism and usage of
   *  simple file reader is not thread safe
   * @see filems::FileReader::read
   * @see filems::SimpleReadingStrategy
   */
  ReadArg read() override { return readingStrategy->read(); };

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Set the path to the file to be read, also opening the input
   *  stream for the new file and updating the strategy. It assures the
   *  previous input file stream is closed before opening the new one.
   * @see filems::FileReader::setPath
   */
  void setPath(std::string const& path) override
  {
    setPath(path);
    ifs.close();
    ifs = std::ifstream(path, openMode);
    makeStrategy();
  }
};

}
}
