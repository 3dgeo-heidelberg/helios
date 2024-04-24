#pragma once

#include <adt/bintree/IBinaryTreeNode.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface extending the Binary Tree node definition to become a
 *  Expression Tree node.
 * Any class providing expression tree node based functionalities must
 *  implement the IExprTreeNode, which implies it must also implement the
 *  IBinaryTreeNode interface.
 * @tparam InputType The input type to be used by the expression tree
 * @tparam OutputType The output type produced by evaluation the expression
 *  tree
 * @see IBinaryTreeNode
 */
template <typename InputType, typename OutputType>
class IExprTreeNode : public IBinaryTreeNode {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    ~IExprTreeNode() override = default;

    // ***  EXPRESSION TREE NODE INTERFACE *** //
    // *************************************** //
    /**
     * @brief Evaluate the given node. In doing so, children nodes will be
     *  evaluated if necessary.
     * @param x The variable \f$x\f$ for the evaluation of the expression tree
     * @return Result obtained after evaluating the node
     */
    virtual OutputType eval(InputType const x) const = 0;
};