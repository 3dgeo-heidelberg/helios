#pragma once

#include <boost/serialization/access.hpp>

#include <functional>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle allocation of multiple instances of the same class
 *  by blocks. It is useful to reduce the number of new calls that, when too
 *  high, might cause a performance bottleneck
 * @tparam Class Type of class to be allocated
 */
template<class Class>
class BlockAllocator
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a BlockAllocator to a stream of byes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the BlockAllocator
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & blockSize;
    ar & blocks;
    ar & lastBlockElements;
    ar & lastBlock;
    ar & _nextBlockSize;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief How many elements per allocated block
   */
  std::size_t blockSize;
  /**
   * @brief Vector of allocated blocks. Each pointer in this vectors points
   *  to the starting element of a block of blockSize elements
   * @see BlockAllocator::lastBlock
   */
  std::vector<Class*> blocks;
  /**
   * @brief The number of already used elements in last block
   * @see BlockAllocator::lastBlock
   */
  std::size_t lastBlockElements;
  /**
   * @brief Pointer to last block in blocks. It is useful to prevent multiple
   *  queries for the same block at blocks vector
   * @see BlockAllocator::blocks
   * @see BlockAllocator::lastBlockElements
   */
  Class* lastBlock;
  /**
   * @brief Compute the next block size. By default it preserves blockSize
   *  but it can be overridden to provide a dynamic behavior for block size.
   */
  std::function<std::size_t(void)> _nextBlockSize;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for BlockAllocator
   * @param blockSize The block size to be used for allocations
   */
  BlockAllocator(std::size_t const blockSize = 256)
    : blockSize(blockSize)
    , lastBlockElements(0)
    , lastBlock(nullptr)
    , _nextBlockSize([&]() -> std::size_t { return blockSize; })
  {
  }
  /**
   * @brief Destructor for BlockAllocator
   */
  virtual ~BlockAllocator() = default;

  // ***  ALLOCATOR METHODS  *** //
  // *************************** //
  /**
   * @brief Obtain the next new allocated object.
   *
   * If no block has been allocated, it handles the allocation of the first
   *  block. If last allocated block is not full, it return the next not yet
   *  used element in the block. If last allocated block is full, it
   *  allocates a new block
   *
   * @return Next new allocated object
   */
  virtual Class* nextNew()
  {
    // Allocate first or next block
    if (lastBlock == nullptr || lastBlockElements == blockSize) {
      blockSize = _nextBlockSize();
      lastBlock = _new();
      blocks.push_back(lastBlock);
      lastBlockElements = 1;
      return lastBlock;
    }

    // Return next element from last allocated block
    Class* next = lastBlock + lastBlockElements;
    ++lastBlockElements;
    return next;
  }

protected:
  /**
   * @brief The allocation itself
   * @return Allocated block
   */
  virtual inline Class* _new() { return new Class[blockSize]; }

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Update next block size function
   * @param f New next block size function
   * @see BlockAllocator::_nextBlockSize
   */
  virtual inline void setNextBlockSize(std::function<size_t(void)> f)
  {
    this->_nextBlockSize = f;
  }
  /**
   * @brief Obtain a vector with pointers to start of allocated blocks
   * @return Vector with pointers to start of allocated blocks
   */
  virtual inline std::vector<Class*> getBlocks() const { return blocks; }
};
