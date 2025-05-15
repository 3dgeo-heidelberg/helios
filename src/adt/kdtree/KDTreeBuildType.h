#pragma once

#include <KDTreeNode.h>

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Build type to wrap data required for recursive building of KDTree
 *  nodes when using a KDTreeFactory based thread pool
 * @see KDTreeFactoryThreadPool
 * @see KDTreeFactory
 */
class KDTreeBuildType
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The parent node
   * @see SimpleKDTreeFactory::buildRecursive
   */
  KDTreeNode* parent;
  /**
   * @brief True if given node is a left child, false otherwise. Notice root
   *  node is not left nor right, thus it must be false for root nodes.
   * @see SimpleKDTreeFactory::buildRecursive
   */
  bool left;
  /**
   * @brief Primitives to build KDTree node from
   * @see SimpleKDTreeFactory::buildRecursive
   */
  vector<Primitive*> primitives;
  /**
   * @brief Depth of node
   * @see SimpleKDTreeFactory::buildRecursive
   */
  int depth;
  /**
   * @brief Index of node at current depth
   * @see SimpleKDTreeFactory::buildRecursive
   */
  int index;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for KDTreeBuildType
   */
  KDTreeBuildType()
    : parent(nullptr)
    , left(false)
    , primitives(0)
    , depth(-1)
    , index(-1)
  {
  }

  /**
   * @brief Constructor for KDTreeBuildType with attributes as arguments
   * @see KDTreeBuildType::parent
   * @see KDTreeBuildType::left
   * @see KDTreeBuildType::primitives
   * @see KDTreeBuildType::depth
   */
  KDTreeBuildType(KDTreeNode* parent,
                  bool const left,
                  vector<Primitive*>& primitives,
                  int const depth,
                  int const index)
    : parent(parent)
    , left(left)
    , primitives(primitives)
    , depth(depth)
    , index(index)
  {
  }

  virtual ~KDTreeBuildType() = default;
};
