#pragma once

#include <LightKDTreeNode.h>

/**
 * @brief Class representing a KDTree node
 */
class KDTreeNode : public LightKDTreeNode
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The axis-aligned boundary of the node
   */
  AABB bound;
  /**
   * @brief The summation of areas for all faces at node boundaries
   */
  double surfaceArea = std::numeric_limits<double>::quiet_NaN();

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for KDTreeNode
   */
  KDTreeNode() {};
  /**
   * @brief Copy constructor for KDTreeNode
   * @param kdtn KDTreeNode to be copy-constructed
   */
  KDTreeNode(KDTreeNode const& kdtn);
  /**
   * @brief Move constructor for KDTreeNode
   * @param kdtn KDTreeNode to be move-constructed
   */
  KDTreeNode(KDTreeNode&& kdtn);
  /**
   * @brief Destructor for KDTreeNode
   */
  ~KDTreeNode() override = default;

  // ***  ASSIGNMENT OPERATORS  *** //
  // ****************************** //
  /**
   * @brief Copy assignment operator for KDTreeNode
   * @param kdtn KDTreeNode to be copy-assigned
   * @return Reference to copied KDTreeNode
   */
  KDTreeNode& operator=(KDTreeNode const& kdtn);
  /**
   * @brief Move assignment operator for KDTreeNode
   * @param kdtn KDTreeNode to be move-assigned
   * @return Reference to moved KDTreeNode
   */
  KDTreeNode& operator=(KDTreeNode&& kdtn);

  // ***   S W A P   *** //
  // ******************* //
  /**
   * @brief Swap attributes of given KDTreeNode and current KDTreeNode
   * @param kdtn KDTreeNode to swap attributes with
   */
  void swap(KDTreeNode& kdtn);
};
