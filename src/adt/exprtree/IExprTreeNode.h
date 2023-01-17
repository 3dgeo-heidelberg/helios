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
 * @tparam NumericType The numeric type to be used by the expression tree
 * @see IBinaryTreeNode
 */
template <typename NumericType>
class IExprTreeNode : public IBinaryTreeNode {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    virtual ~IExprTreeNode() = default;

    // ***  EXPRESSION TREE NODE INTERFACE *** //
    // *************************************** //
    /**
     * @brief Evaluate the given node. In doing so, children nodes will be
     *  evaluated if necessary.
     * @param t The variable \f$t\f$ for the evaluation of the univariate
     *  expression tree
     * @return Result obtained after evaluating the node
     */
    virtual NumericType eval(NumericType const t) const = 0;
};