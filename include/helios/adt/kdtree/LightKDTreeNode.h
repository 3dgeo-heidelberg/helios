#pragma once

#include <string>
#include <vector>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
// This works around a known issue in boost:
// https://github.com/boostorg/serialization/issues/315
#ifdef BOOST_NO_EXCEPTIONS
#include <boost/throw_exception.hpp>
#endif

#include <boost/serialization/vector.hpp>

#include <helios/adt/bintree/BinaryTreeBreadthIterator.h>
#include <helios/adt/bintree/BinaryTreeDepthIterator.h>
#include <helios/adt/bintree/BinaryTreeFastBreadthIterator.h>
#include <helios/adt/bintree/BinaryTreeFastDepthIterator.h>
#include <helios/adt/bintree/IBinaryTreeNode.h>

#include <helios/scene/primitives/AABB.h>
#include <helios/scene/primitives/Primitive.h>

// Including primitives below is necessary for serialization
#include <helios/scene/primitives/DetailedVoxel.h>
#include <helios/scene/primitives/Triangle.h>
#include <helios/scene/primitives/Voxel.h>

/**
 * @brief Class representing a light KDTree node. It is, the basic
 *  representation of a KDTree node with uses least possible data.
 */
class LightKDTreeNode : public IBinaryTreeNode
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a LightKDTreeNode to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of byes
   * @param version Version number for the LightKDTreeNode
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    // Register classes derived from Primitive
    ar.template register_type<Vertex>();
    ar.template register_type<AABB>();
    ar.template register_type<Triangle>();
    ar.template register_type<Voxel>();
    ar.template register_type<DetailedVoxel>();

    boost::serialization::void_cast_register<LightKDTreeNode,
                                             IBinaryTreeNode>();
    ar & left;
    ar & right;
    ar & splitPos;
    ar & splitAxis;
    ar & primitives;
  }

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Pointer to node at left side on space partition. Can be nullptr
   *  if there is no left side node
   */
  LightKDTreeNode* left = nullptr;
  /**
   * @biref Pointer to node at right side on space partition. Can be nullptr
   *  if there is no right side node
   */
  LightKDTreeNode* right = nullptr;
  /**
   * @brief Point position at corresponding split axis
   */
  double splitPos = 0;
  /**
   * @brief Space axis to consider at current depth
   */
  int splitAxis = 0;
  /**
   * @brief Vector of primitives associated with the node
   */
  std::shared_ptr<std::vector<Primitive*>> primitives;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for LightKDTreeNode
   */
  LightKDTreeNode() = default;
  /**
   * @brief Copy constructor for LightKDTreeNode
   * @param kdtn LightKDTreeNode to be copy-constructed
   */
  LightKDTreeNode(LightKDTreeNode const& kdtn);
  /**
   * @brief Move constructor for LightKDTreeNode
   * @param kdtn LightKDTreeNode to be move-constructed
   */
  LightKDTreeNode(LightKDTreeNode&& kdtn);
  /**
   * @brief Destructor for LightKDTreeNode
   */
  ~LightKDTreeNode() override
  {
    delete left;
    delete right;
  }

  // ***  ASSIGNMENT OPERATORS  *** //
  // ****************************** //
  /**
   * @brief Copy assignment operator for LightKDTreeNode
   * @param kdtn LightKDTreeNode to be copy-assigned
   * @return Reference to copied LightKDTreeNode
   */
  LightKDTreeNode& operator=(LightKDTreeNode const& kdtn);
  /**
   * @brief Move assignment operator for LightKDTreeNode
   * @param kdtn LightKDTreeNode to be move-assigned
   * @return Reference to moved LightKDTreeNode
   */
  LightKDTreeNode& operator=(LightKDTreeNode&& kdtn);

  // ***   S W A P   *** //
  // ******************* //
  /**
   * @brief Swap attributes of given LightKDTreeNode and current
   *  LightKDTreeNode
   * @param kdtn LightKDTreeNode to swap attributes with
   */
  void swap(LightKDTreeNode& kdtn);

  // ***  BINARY TREE INTERFACE  *** //
  // ******************************* //
  /**
   * @see IBinaryTree::getLeftChild
   */
  LightKDTreeNode* getLeftChild() const override { return left; }
  /**
   * @see IBinaryTree::getRightChild
   */
  LightKDTreeNode* getRightChild() const override { return right; }

  // ***  ITERATION METHODS  *** //
  // *************************** //
  /**
   * @brief Build a depth iterator starting at this node
   * @return Depth iterator starting at this node
   * @see BinaryTreeDepthIterator
   */
  inline BinaryTreeDepthIterator<LightKDTreeNode> buildDepthIterator(
    int const depth = 0)
  {
    return { this, depth };
  }
  /**
   * @brief Build a fast depth iterator starting at this node
   * @return Fast depth iterator starting at this node
   * @see BinaryTreeFastDepthIterator
   */
  inline BinaryTreeFastDepthIterator<LightKDTreeNode> buildFastDepthIterator()
  {
    return { this };
  }
  /**
   * @brief Build a breadth iterator starting at this node
   * @return Breadth iterator starting at this node
   * @see BinaryTreeBreadthIterator
   */
  inline BinaryTreeBreadthIterator<LightKDTreeNode> buildBreadthIterator(
    int const depth = 0)
  {
    return { this, depth };
  }
  /**
   * @brief Build a fast breadth iterator starting at this node
   * @return Fast breadth iterator starting at this node
   * @see BinaryTreeFastBreadthIterator
   */
  inline BinaryTreeFastBreadthIterator<LightKDTreeNode>
  buildFastBreadthIterator()
  {
    return { this };
  }
};
