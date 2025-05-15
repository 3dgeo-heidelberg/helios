#pragma once

#include <ExpressionRegisterHandler.h>
#include <UnivarExprTreeNode.h>
#include <util/HeliosException.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a univariate expression tree node with registers.
 *
 * The Register Univariate Expression Tree Node supports univariate expressions
 *  that can access the software's internal variables.
 * The registers are represented by a string starting with "ER" and followed
 *  by a number that indicates the register index. The mapping between
 *  expression registers and internal variables is done by the
 *  ExpressionRegisterHandler
 *
 * @tparam NumericType The numeric type to be used by the univariate expression
 *  tree with registers. It must be contained or equal to the reals (
 *  \f$\subseteq \mathbb{R}\f$)
 * @see ExpressionRegisterHandler
 */
template<typename NumericType>
class RegUnivarExprTreeNode : public UnivarExprTreeNode<NumericType>
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Pointer to the handler for expression registers
   * @see ExpressionRegisterHandler
   * @see RegUnivarExprTree::erh
   */
  std::shared_ptr<ExpressionRegisterHandler<NumericType>> erh;
  /**
   * @brief The index of the register in the ExpressionRegisterHandler
   *  corresponding to this node
   * @see ExpressionRegisterHandler
   * @see RegUnivarExprTreeNode::erh
   */
  size_t const regIdx = 0;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Construct a register-based univariate expression tree node
   * @param erh The expression register handler
   * @param left The left child node
   * @param right The right child node
   * @see UnivarExprTreeNode
   */
  RegUnivarExprTreeNode(
    std::shared_ptr<ExpressionRegisterHandler<NumericType>> erh,
    size_t const regIdx = 0,
    RegUnivarExprTreeNode* left = nullptr,
    RegUnivarExprTreeNode* right = nullptr)
    : UnivarExprTreeNode<NumericType>(left, right)
    , erh(erh)
    , regIdx(regIdx)
  {
  }
  /**
   * @brief UNSUPPORTED CONSTRUCTOR (DONT USE)
   *
   * This constructor must be defined for compatibility with the
   *  UnivarExprTreeStringFactory::newExprTree method but it MUST NOT be
   *  used because it is not supported
   */
  RegUnivarExprTreeNode(RegUnivarExprTreeNode* left = nullptr,
                        RegUnivarExprTreeNode* right = nullptr)
    : UnivarExprTreeNode<NumericType>(left, right)
  {
    throw HeliosException("Building a RegUnivarExprTreeNode without an "
                          "ExpressionRegisterHandler is not supported");
  }
  virtual ~RegUnivarExprTreeNode() = default;

  // ***  EXPRESSION TREE NODE INTERFACE  *** //
  // **************************************** //
  /**
   * @brief Extend the univariate expression tree node eval function to
   *     account for registers
   * @see UnivarExprTreeNode::eval
   * @see IExprTreeNode::eval
   */
  NumericType eval(NumericType const t) const override
  {
    // Handle register as extension
    if (isRegister())
      return erh->getRegByVal(regIdx);
    // If no register evaluation, delegate upon Univariate Expression Tree
    return UnivarExprTreeNode<NumericType>::eval(t);
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Check whether the node is a register (true) or not (false)
   * @return True if node is a register, false otherwise
   */
  inline bool isRegister() const
  {
    return UnivarExprTreeNode<NumericType>::symbolType ==
           UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION;
  }
};
