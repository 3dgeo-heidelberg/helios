#pragma once

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
  // *********************** //

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
