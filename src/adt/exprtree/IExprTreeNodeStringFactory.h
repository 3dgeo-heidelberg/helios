#pragma once

#include <IExprTreeNode.h>

#include <memory>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface defining the functions that must be provided by any
 *  concrete implementation of a factory that builds expression trees from a
 *  given string
 * @tparam InputType The input type for built expression trees
 * @tparam OutputTpye The output type for built expression trees
 * @see IExprTreeNode
 */
template<typename InputType, typename OutputType>
class IExprTreeNodeStringFactory
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  virtual ~IExprTreeNodeStringFactory() = default;

  // ***  EXPRESSION TREE NODE STRING FACTORY INTERFACE  *** //
  // ******************************************************* //
  /**
   * @brief Make an expression tree from given expression as string
   * @param expr The expression given as a string
   * @return Built expression tree
   */
  virtual IExprTreeNode<InputType, OutputType>* make(
    std::string const& expr) = 0;
  /**
   * @brief Like IExprTreeNodeStringFactory::make but returning a shared
   *  pointer instead of a raw pointer
   * @see IExprTreeNodeStringFactory::make
   */
  inline std::shared_ptr<IExprTreeNode<InputType, OutputType>> makeShared(
    std::string const& expr)
  {
    return std::shared_ptr<IExprTreeNode<InputType, OutputType>>(make(expr));
  }
};
