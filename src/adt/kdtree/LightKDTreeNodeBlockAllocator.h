#pragma once

// This works around a known issue in boost:
// https://github.com/boostorg/serialization/issues/315
#ifdef BOOST_NO_EXCEPTIONS
#include <boost/throw_exception.hpp>
#endif
#include <boost/serialization/void_cast.hpp>

#include <BlockAllocator.h>
#include <LightKDTreeNode.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Block allocator for LightKDTreeNode instances
 * @see BlockAllocator
 * @see LightKDTreeNode
 */
class LightKDTreeNodeBlockAllocator : public BlockAllocator<LightKDTreeNode>
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a LightKDTreeNodeBlockAllocator to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the LightKDTreeNodeBlockAllocator
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::void_cast_register<LightKDTreeNodeBlockAllocator,
                                             BlockAllocator>();
    ar& boost::serialization::base_object<BlockAllocator>(*this);
  }

public:
  /**
   * @brief Default constructor for LightKDTreeNodeBlockAllocator
   * @see BlockAllocator::BlockAllocator(size_t const)
   */
  LightKDTreeNodeBlockAllocator(size_t const blockSize = 256)
    : BlockAllocator(blockSize)
  {
  }
  /**
   * @brief Destructor for LightKDTreeNodeBlockAllocator
   */
  virtual ~LightKDTreeNodeBlockAllocator() = default;
};
