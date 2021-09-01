#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 *
 * @tparam NodeType Type of node
 *
 * @brief Binary tree node interface that must be implemented by any class
 *  providing binary tree node based functionalities
 */
template <typename NodeType>
class IBinaryTreeNode {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    virtual ~IBinaryTreeNode() = default;

    // ***  BINARY TREE INTERFACE  *** //
    // ******************************* //
    /**
     * @brief Obtain the left child of current node
     * @return Left child
     */
    virtual NodeType * getLeftChild() const = 0;
    /**
     * @brief Obtain the right child of current node
     * @return Right child
     */
    virtual NodeType * getRightChild() const = 0;
    /**
     * @brief Check whether current node is a leaf node (true) or not (false)
     * @return True if current node is a leaf node, false otherwise
     */
    virtual bool isLeafNode()
    {return getLeftChild()==nullptr && getRightChild()==nullptr;}

};
