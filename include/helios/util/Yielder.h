#pragma once

#include <helios/filems/facade/FMSWriteFacade.h>

#include <cstdlib>
#include <mutex>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class representing a yielder. It is, an object which can be
 *  used to accumulate inputs until either a yield is explicitly deleted or
 *  the yield condition is triggered, probably after a push request
 * @tparam T The type of object handled by the yielder
 */
template<typename T>
class Yielder
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The mutex to handle concurrent push backs to the buffer vector
   *  and the yielding operation itself
   */
  std::mutex mtx;
  /**
   * @brief The number of elements that can be buffered before forcing
   *  the yield operation
   */
  std::size_t bufferSize;
  /**
   * @brief Where the elements are stored
   */
  std::vector<T> buffer;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the abstract yielder
   * @see Yielder::bufferSize
   */
  Yielder(std::size_t bufferSize = 256) { setBufferSize(bufferSize); }
  virtual ~Yielder() = default;

  // ***  YIELD METHODS  *** //
  // *********************** //
  /**
   * @brief Make the yielder flush its elements so the output is
   *  performed
   */
  inline void yield()
  {
    // Copy and clear in critical region
    mtx.lock();
    vector<T> copy = copyBuffer();
    buffer.clear();
    mtx.unlock();
    // Digest temporal copy
    digest(copy);
  }
  /**
   * @brief Push the element into the yielder. The element could be
   *  simply accumulated or either it could be directly written to file
   *  depending on yielder status at push time
   * @param elem Element to be pushed into the yielder
   */
  inline void push(T const& elem)
  {
    mtx.lock();
    buffer.push_back(elem);
    if (buffer.size() >= bufferSize) {
      // Copy what must be written before releasing the mutex
      vector<T> copy = copyBuffer();
      // Empty buffer before releasing the mutex
      buffer.clear();
      mtx.unlock();
      // Digest temporal copy
      digest(copy);
    } else {
      mtx.unlock();
    }
  }

  /**
   * @brief Abstract method that must be overridden by any concrete Yielder
   *  implementation to specify how a yield output must be handled/digested
   */
  virtual void digest(vector<T>& copy) = 0;
  /**
   * @brief Make a copy of the buffer with its current state
   * @return The copy of the buffer at its current state
   */
  virtual vector<T> copyBuffer() const { return vector<T>(buffer); }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Set the buffer size of the yielder, effectively
   * @param bufferSize New buffer size for the yielder
   */
  inline void setBufferSize(size_t const bufferSize)
  {
    this->bufferSize = bufferSize;
  }
  /**
   * @brief Obtain the current buffer size of the yielder
   * @return The buffer size of the yielder
   */
  inline size_t getBufferSize() const { return bufferSize; }
};
