#ifdef DATA_ANALYTICS
#pragma once

#include <HDA_OfstreamWrapper.h>

#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace helios {
namespace analytics {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The HeliosDataAnalytics record buffer. It is, a class which handles
 *  the storage of records in memory and its writing to the corresponding file.
 *  Thus, the record buffer is in fact a write buffer.
 * @tparam T The type of element stored at the buffer.
 */
template<typename T>
class HDA_RecordBuffer
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The buffer itself
   */
  std::vector<T> buff;
  /**
   * @brief The maximum number of elements supported by the buffer
   */
  size_t maxSize;
  /**
   * @brief The path to the output file were records are written
   */
  std::string outpath;
  /**
   * @brief The wrapped output stream to write the contents of the buffer
   */
  HDA_OfstreamWrapper ofsw;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a HDA_RecordBuffer to handle the storage and writing of
   *  records.
   * @see HDA_RecordBuffer::outpath
   * @see HDA_RecordBuffer::maxSize
   */
  HDA_RecordBuffer(std::string const& outpath,
                   size_t const maxSize = 256,
                   std::string const& sep = ",",
                   bool const vectorial = false)
    : maxSize(maxSize)
    , outpath(outpath)
    , ofsw(outpath, sep)
  {
  }

  virtual ~HDA_RecordBuffer()
  {
    if (isOpen())
      close();
  }

  // ***  RECORD BUFFER METHODS  *** //
  // ******************************* //
  /**
   * @brief Check whether the output stream of the record buffer is opened or
   *  not
   * @return True if the output stream is opened, false otherwise
   */
  inline bool isOpen() { return ofsw.is_open(); }
  /**
   * @brief Method to write the elements of the buffer to a file.
   */
  inline void write()
  {
    for (T const& elem : buff)
      ofsw << elem;
  }
  /**
   * @brief Write all the contents of the buffer and then make it empty
   */
  inline void flush()
  {
    if (buff.size() > 0) {
      write();
      buff.clear();
    }
  }
  /**
   * @brief Write all the contents of the buffer, make it empty and close the
   *  output stream
   */
  inline void close()
  {
    flush();
    if (isOpen())
      ofsw.close();
  }
  /**
   * @brief Insert given element in the buffer. If the buffer is full, it
   *  will be automatically flushed before inserting the new element.
   * @param elem The element to be inserted
   * @see HDA_RecordBuffer::flush
   */
  inline void push(T const& elem)
  {
    if (buff.size() >= maxSize)
      flush();
    buff.push_back(elem);
  }
};

}
}

#endif
