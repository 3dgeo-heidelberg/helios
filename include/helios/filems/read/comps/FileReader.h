#pragma once

#include <helios/filems/read/strategies/ReadingStrategy.h>

#include <memory>
#include <string>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class defining the fundamentals of any file reader
 * @tparam ReadType Type of what is read from file
 */
template<typename ReadType>
class FileReader
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Path to the file to be read
   */
  std::string path;
  /**
   * @brief The reading strategy to be used by the file reader
   * @see filems::ReadingStrategy
   */
  std::shared_ptr<ReadingStrategy<ReadType>> readingStrategy = nullptr;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for file reader
   * @see filems::FileReader::path
   */
  FileReader(std::string const& path)
    : path(path)
  {
  }
  virtual ~FileReader() = default;

  // ***  READ METHODS  *** //
  // ********************** //
  /**
   * @brief Read from file
   * @return What has been read from file
   */
  virtual ReadType read() = 0;

protected:
  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Build the strategy for the file reader
   */
  virtual void makeStrategy() = 0;

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the path to the file to be read
   * @return Path to the file to be read
   * @see filems::FileReader::path
   */
  virtual std::string getPath() const { return path; };
  /**
   * @brief Set the path to the file to be read
   * @param path Path to the file to be read
   * @see filems::FileReader::path
   */
  virtual void setPath(std::string const& path) { this->path = path; }
};

}
}
