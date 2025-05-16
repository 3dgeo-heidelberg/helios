#pragma once

#include <filems/read/exceptions/EndOfReadingException.h>
#include <filems/read/strategies/ReadingStrategy.h>

#include <queue>

namespace helios {
namespace filems {

using std::queue;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @tparam ReadArg Type of what is being read
 *
 * @brief Class defining a strategy that provides support for buffering of
 *  consecutive readings
 */
template<typename ReadArg>
class BufferedReadingStrategy : public ReadingStrategy<ReadArg>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The maximum number of ReadArg type elements that a buffer is
   *  allowed to hold
   */
  std::size_t bufferSize;
  /**
   * @brief The buffer where multiple consecutive reads are stored
   */
  ReadArg* buffer = nullptr;
  /**
   * @brief The reading strategy that is being extended with buffer support
   */
  ReadingStrategy<ReadArg>& readingStrategy;

  /**
   * @brief The cached index defining the state of the buffer
   */
  std::size_t cachedIndex = 0;
  /**
   * @brief The cached index defining the next to the greatest
   *  admissible index for the current state of the buffer
   */
  std::size_t cachedMaxIndex = 0;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for buffered reading strategy
   * @see BufferedReadingStrategy::readingStrategy
   */
  BufferedReadingStrategy(ReadingStrategy<ReadArg>& readingStrategy,
                          std::size_t const bufferSize = 256)
    : bufferSize(bufferSize)
    , readingStrategy(readingStrategy)
  {
    buffer = new ReadArg[bufferSize];
  }
  virtual ~BufferedReadingStrategy() { delete[] buffer; }

  // ***  READING STRATEGY METHODS  *** //
  // ********************************** //
  /**
   * @brief Take until buffer is empty, read always that buffer needs to be
   *  fulfilled
   * @return Either what has been read from input reading strategy or throw
   *  an exception if it passes through it
   */
  ReadArg read() override
  {
    // When buffer is not empty : read next element (FIFO)
    if (!isBufferEmpty()) {
      std::size_t const bufferIndex = cachedIndex;
      ++cachedIndex;
      --cachedMaxIndex;
      return buffer[bufferIndex];
    }
    // When buffer is empty : populate it again by reading strategy
    try {
      while (cachedMaxIndex < bufferSize) {
        buffer[cachedMaxIndex] = readingStrategy.read();
        ++cachedMaxIndex;
      }
    } catch (EndOfReadingException& eorex) {
    } // Handle EndOfReadingExceptions
    // If buffer stills empty after trying to populate it
    if (isBufferEmpty()) { // Throw EndOfReadingException
      throw EndOfReadingException();
    }
    // If not, return next from buffer
    cachedIndex = 1;
    --cachedMaxIndex;
    return buffer[0];
  }

protected:
  // ***  BUFFER HANDLING METHODS  *** //
  // ********************************* //
  /**
   * @brief Check whether the buffer is empty or not
   * @return True if the buffer is empty, false otherwise
   */
  virtual bool isBufferEmpty() const { return cachedMaxIndex == 0; }

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the buffer size
   * @return The buffer size
   * @see filems::BufferedReadingStrategy::bufferSize
   * @see filems::BufferedReadingStrategy::setBufferSize
   */
  virtual inline std::size_t getBufferSize() { return bufferSize; }
  /**
   * @brief Set the buffer size and update the buffer to fit
   * @param bufferSize The new buffer size
   * @see filems::BufferedReadingStrategy::bufferSize
   * @see filems::BufferedReadingStrategy::getBufferSize
   */
  virtual inline void setBufferSize(std::size_t const bufferSize)
  {
    this->bufferSize = bufferSize;
    delete[] this->buffer;
    this->buffer = new ReadArg[bufferSize];
  }
};

}
}
