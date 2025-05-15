#pragma once

#include <IBinaryTreeNode.h>
#include <ITreeIterator.h>

#include <deque>
#include <iterator>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Fast breadth first iterator for binary tree like nodes
 *
 * Breadth first node visiting order is illustrated below:
 * <table>
 * <tr> <td colspan=20 align=center>0</td> </tr>
 * <tr>
 *  <td colspan=5></td>
 *  <td align=center>1</td>
 *  <td colspan=10></td>
 *  <td align=center>2</td>
 *  <td colspan=3></td>
 * </tr>
 * <tr>
 *  <td colspan=3></td>
 *  <td>3</td>
 *  <td colspan=3></td>
 *  <td>4</td>
 *  <td colspan=6></td>
 *  <td>5</td>
 *  <td colspan=3></td>
 *  <td>6</td>
 *  <td colspan=3></td>
 * </tr>
 * </table>
 *
 * @tparam NodeType Type of binary tree node. It must correspond to a class
 *  which extends IBinaryTreeNode interface
 * @see IBinaryTreeNode
 */
template<typename NodeType>
class BinaryTreeFastBreadthIterator : public ITreeIterator<NodeType*>
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a BinaryTreeFastBreadthIterator to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the BinaryTreeFastBreadthIterator
   */
  template<class Archive>
  void serialize(Archive& ar, unsigned int const version)
  {
    boost::serialization::base_object<BinaryTreeFastBreadthIterator<NodeType>,
                                      ITreeIterator<NodeType*>>();
    ar& boost::serialization::base_object<ITreeIterator<NodeType*>>(*this);
    ar & pendingNodes;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Double ended queue used as a queue to handles nodes visiting
   */
  deque<IBinaryTreeNode*> pendingNodes;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for binary tree fast breadth iterator
   */
  BinaryTreeFastBreadthIterator() = default;
  /**
   * @brief Construct a binary tree fast breadth iterator calling the start
   *  method with given node
   * @param node Node to start breadth iterating from
   * @see BinaryTreeFastBreadthIterator::start
   */
  BinaryTreeFastBreadthIterator(NodeType* node)
    : BinaryTreeFastBreadthIterator()
  {
    start(node);
  }
  virtual ~BinaryTreeFastBreadthIterator() {}

  // ***  TREE ITERATOR INTERFACE  *** //
  // ********************************* //
  /**
   * @brief Start the iterator so the first visited node will be given one.
   * It is, when calling next, given node will be returned
   * @param node Node to start breadth iterating from
   * @see ITreeIterator::start
   */
  inline void start(NodeType* node) override
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
   * @brief Obtain the next node according to breadth iteration criterion
   * @return Next node according to breadth iteration criterion
   * @see ITreeIterator::next
   */
  inline NodeType* next() override
  {
    IBinaryTreeNode* node = pendingNodes.front();
    pendingNodes.pop_front();
    if (node->getLeftChild() != nullptr)
      pendingNodes.push_back(node->getLeftChild());
    if (node->getRightChild() != nullptr)
      pendingNodes.push_back(node->getRightChild());
    return (NodeType*)node;
  }
};
