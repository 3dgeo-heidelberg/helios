#pragma once

// This works around a known issue in boost:
// https://github.com/boostorg/serialization/issues/315
#ifdef BOOST_NO_EXCEPTIONS
#include <boost/throw_exception.hpp>
#endif
#include <boost/serialization/access.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing an iterable tree node. It is a wrapper
 *  for a given tree node type which handles some extra features that can be
 *  useful during iteration such as depth level
 * @tparam NodeType Tree node type being wrapped
 */
template<typename NodeType>
class IterableTreeNode
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a IterableTreeNode to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the IterableTreeNode
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & node;
    ar & depth;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Tree node being wrapped
   */
  NodeType* node;
  /**
   * @brief Depth of tree node being wrapped
   */
  int depth;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Construct an iterable tree node
   * @param node Tree node being wrapped
   * @param depth Node depth inside the tree
   * @see IterableTreeNode::node
   * @see IterableTreeNode::depth
   */
  IterableTreeNode(NodeType* node, int depth = 0)
    : node(node)
    , depth(depth)
  {
  }
  virtual ~IterableTreeNode() {}

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the tree node being wrapped
   * @return Tree node being wrapped
   * @see IterableTreeNode::node
   */
  inline NodeType* getNode() const { return node; }
  /**
   * @brief Set the tree node being wrapped
   * @param node New tree node being wrapped
   * @see IterableTreeNode::node
   */
  inline void setNode(NodeType* node) { this->node = node; }
  /**
   * @brief Set both the tree node being wrapped and its depth
   * @param node New tree node being wrapped
   * @param depth New depth for tree node being wrapped
   * @see IterableTreeNode::setNode(NodeType *)
   * @see IterableTreeNode::setDepth(int const)
   */
  inline void setNode(NodeType* node, int const depth)
  {
    setNode(node);
    setDepth(depth);
  }
  /**
   * @brief Obtain the depth of wrapped tree node
   * @return Depth of wrapped tree node
   * @see IterableNode::depth
   */
  inline int getDepth() const { return depth; }
  /**
   * @brief Set the depth of tree node being wrapped
   * @param depth New depth for tree node being wrapped
   * @see IterableNode::depth
   */
  inline void setDepth(int const depth) { this->depth = depth; }
};
