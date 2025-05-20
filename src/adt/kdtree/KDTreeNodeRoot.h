#pragma once

#include <KDTreeNode.h>

#include <vector>

/**
 * @brief Class representing the root node of a KDTree
 *
 * @see KDTreeNode
 */
class KDTreeNodeRoot : public KDTreeNode
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a KDTreeNodeRoot to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the KDTreeNodeRoot
   */
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::void_cast_register<KDTreeNodeRoot, KDTreeNode>();
    ar& boost::serialization::base_object<KDTreeNode>(*this);
    ar & stats_maxNumPrimsInLeaf;
    ar & stats_minNumPrimsInLeaf;
    ar & stats_maxDepthReached;
    ar & stats_numInterior;
    ar & stats_numLeaves;
    ar & stats_totalCost;
  }

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Maximum number of primitives considering all leaves
   */
  int stats_maxNumPrimsInLeaf;
  /**
   * @brief Minimum number of primitives considering all leaves
   */
  int stats_minNumPrimsInLeaf;
  /**
   * @brief Maximum depth of the KDTree
   */
  int stats_maxDepthReached;
  /**
   * @brief Number of interior nodes in the KDTree
   */
  int stats_numInterior;
  /**
   * @brief Number of leaf nodes in the KDTree
   */
  int stats_numLeaves;
  /**
   * @brief The total cost of the tree. It changes depending on tree building
   *  strategy. It might be NaN if tree building strategy is not based on
   *  computing any cost
   */
  double stats_totalCost;
  /**
   * @brief Vector of pointers to the start of each allocated block of
   *  LightKDTreeNode. If it is empty, it means the tree has not been
   *  lightened, thus its allocation is not block based.a
   * @see KDTreeNodeRoot::blocksSize
   */
  std::vector<LightKDTreeNode*> blocks;
  /**
   * @brief The size (number of nodes) for each allocated block, if any
   * @see KDTreeNodeRoot::blocks
   */
  std::vector<std::size_t> blocksSize;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for KDTree root node
   */
  KDTreeNodeRoot()
    : stats_maxNumPrimsInLeaf(0)
    , stats_minNumPrimsInLeaf(std::numeric_limits<int>::max())
    , stats_maxDepthReached(0)
    , stats_numInterior(0)
    , stats_numLeaves(0)
    , stats_totalCost(0.0)
  {
  }

  ~KDTreeNodeRoot() override
  {
    // Handle release when nodes have been allocated in blocks
    if (!blocks.empty()) {
      // Root direct children to nullptr to prevent double free
      left = nullptr;
      right = nullptr;

      // Free each allocated block
      size_t const numBlocks = blocks.size();
      for (size_t i = 0; i < numBlocks; ++i) {
        size_t const numNodes = blocksSize[i];
        LightKDTreeNode* baseBlock = blocks[i];
        // All children to nullptr to prevent double free
        for (size_t j = 0; j < numNodes; ++j) {
          LightKDTreeNode* block = baseBlock + j;
          block->left = nullptr;
          block->right = nullptr;
        }
        delete[] baseBlock;
      }
    }
  }
};
