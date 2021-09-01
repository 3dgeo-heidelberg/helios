#pragma once

#include <IBinaryTreeNode.h>
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
 * @tparam NodeType Type of binary tree node
 *
 * @see BinaryTreeFastDepthIterator
 * @see IterableTreeNode
 */
template <typename NodeType>
class BinaryTreeDepthIterator{
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
    void serialize(Archive &ar, const unsigned int version){
        ar &pendingNodes;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Double ended queue used as a stack to handle nodes visiting
     */
    deque<IterableTreeNode<IBinaryTreeNode<NodeType>>> pendingNodes;

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
     * @see BinaryTreeDepthIterator::start
     */
    BinaryTreeDepthIterator(NodeType *node, int const depth=0) :
        BinaryTreeDepthIterator()
    {start(node, depth);}
    /**
     * @brief Construct a binary tree depth iterator calling the start method
     *  with given iterable tree node
     * @param node Iterable tree node to start depth iterating from
     * @see BinaryTreeDepthIterator::start
     */
    BinaryTreeDepthIterator(IterableTreeNode<IBinaryTreeNode<NodeType>> node)
        : BinaryTreeDepthIterator()
    {start(node);}
    virtual ~BinaryTreeDepthIterator() {}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see start(IterableTreeNode<IBinaryTreeNode<NodeType>> *)
     */
    inline void start(NodeType *node, int const depth=0){
        start(IterableTreeNode<IBinaryTreeNode<NodeType>>(node, depth));
    }
    /**
     * @brief Start the iterator so the first visited node will be given one.
     *  It is, when calling next, given node will be returned
     * @param node Node to start depth iterating from
     */
    inline void start(IterableTreeNode<IBinaryTreeNode<NodeType>> node){
        pendingNodes.clear();
        pendingNodes.push_back(node);
    }
    /**
     * @brief Check if the iterator has more nodes to visit (true) or not
     *  (false)
     * @return True if there are nodes left to be visited, false otherwise
     */
    inline bool hasNext() const {return !pendingNodes.empty();}
    /**
     * @brief Obtain the next node according to depth iteration criterion
     * @return Next node according to depth iteration criterion
     */
    inline IterableTreeNode<IBinaryTreeNode<NodeType>> next(){
        IterableTreeNode<IBinaryTreeNode<NodeType>> node = pendingNodes.back();
        pendingNodes.pop_back();
        if(node.getNode()->getRightChild() != nullptr)
            pendingNodes.push_back(
                IterableTreeNode<IBinaryTreeNode<NodeType>>(
                    node.getNode()->getRightChild(),
                    node.getDepth()+1
                )
            );
        if(node.getNode()->getLeftChild() != nullptr)
            pendingNodes.push_back(
                IterableTreeNode<IBinaryTreeNode<NodeType>>(
                    node.getNode()->getLeftChild(),
                    node.getDepth()+1
                )
            );
        return node;
    }
};
