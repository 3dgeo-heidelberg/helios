#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 *
 * @brief Binary tree node interface that must be implemented by any class
 *  providing binary tree node based functionalities
 */
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
    virtual IBinaryTreeNode * getLeftChild() const = 0;
    /**
     * @brief Obtain the right child of current node
     * @return Right child
     */
    virtual IBinaryTreeNode * getRightChild() const = 0;
    /**
     * @brief Check whether current node is a leaf node (true) or not (false)
     * @return True if current node is a leaf node, false otherwise
     */
    virtual bool isLeafNode() const
    {return getLeftChild()==nullptr && getRightChild()==nullptr;}

};
