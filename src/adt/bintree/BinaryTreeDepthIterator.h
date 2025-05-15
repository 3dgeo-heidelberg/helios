#pragma once

#include <IBinaryTreeNode.h>
#include <ITreeIterator.h>
#include <IterableTreeNode.h>

#include <deque>
#include <iterator>

using std::deque;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Like fast depth first iterator but wrapping tree nodes inside a
 *  IterableTreeNode instance
 *
 * @tparam NodeType Type of binary tree node. It must correspond to a class
 *  which extends IBinaryTreeNode interface
 *
 * @see BinaryTreeFastDepthIterator
 * @see IterableTreeNode
 * @see IBinaryTreeNode
 */
template<typename NodeType>
class BinaryTreeDepthIterator
  : public ITreeIterator<IterableTreeNode<IBinaryTreeNode>>
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a BinaryTreeDepthIterator to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the BinaryTreeDepthIterator
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::base_object<
      BinaryTreeDepthIterator<NodeType>,
      ITreeIterator<IterableTreeNode<IBinaryTreeNode>>>();
    ar& boost::serialization::base_object<
      ITreeIterator<IterableTreeNode<IBinaryTreeNode>>>(*this);
    ar & pendingNodes;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Double ended queue used as a stack to handle nodes visiting
   */
  deque<IterableTreeNode<IBinaryTreeNode>> pendingNodes;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for binary tree depth iterator
   */
  BinaryTreeDepthIterator() = default;
  /**
   * @brief Construct a binary tree depth iterator calling the start
   *  method with given node
   * @param node Node to start depth iterating from
   * @param depth The depth at which given node is said to be located
   * @see BinaryTreeDepthIterator::start
   */
  BinaryTreeDepthIterator(NodeType* node, int const depth = 0)
    : BinaryTreeDepthIterator()
  {
    start(node, depth);
  }
  /**
   * @brief Construct a binary tree depth iterator calling the start method
   *  with given iterable tree node
   * @param node Iterable tree node to start depth iterating from
   * @see BinaryTreeDepthIterator::start
   */
  BinaryTreeDepthIterator(IterableTreeNode<IBinaryTreeNode> node)
    : BinaryTreeDepthIterator()
  {
    start(node);
  }
  virtual ~BinaryTreeDepthIterator() {}

  // ***  TREE ITERATOR INTERFACE  *** //
  // ********************************* //
  /**
   * @param depth The depth at which given node is said to be located
   * @see start(IterableTreeNode<IBinaryTreeNode>)
   */
  inline void start(NodeType* node, int const depth = 0)
  {
    start(IterableTreeNode<IBinaryTreeNode>(node, depth));
  }
  /**
   * @brief Start the iterator so the first visited node will be given one.
   *  It is, when calling next, given node will be returned
   * @param node Node to start depth iterating from
   * @see ITreeIterator::start
   */
  inline void start(IterableTreeNode<IBinaryTreeNode> node) override
  {
    pendingNodes.clear();
    pendingNodes.push_back(node);
  }
  /**
   * @brief Check if the iterator has more nodes to visit (true) or not
   *  (false)
   * @return True if there are nodes left to be visited, false otherwise
   * @see ITreeIterator::hasNext
   */
  inline bool hasNext() const override { return !pendingNodes.empty(); }
  /**
   * @brief Obtain the next node according to depth iteration criterion
   * @return Next node according to depth iteration criterion
   * @see ITreeIterator::next
   */
  inline IterableTreeNode<IBinaryTreeNode> next() override
  {
    IterableTreeNode<IBinaryTreeNode> node = pendingNodes.back();
    pendingNodes.pop_back();
    if (node.getNode()->getRightChild() != nullptr)
      pendingNodes.emplace_back(node.getNode()->getRightChild(),
                                node.getDepth() + 1);
    if (node.getNode()->getLeftChild() != nullptr)
      pendingNodes.emplace_back(node.getNode()->getLeftChild(),
                                node.getDepth() + 1);
    return node;
  }
};
