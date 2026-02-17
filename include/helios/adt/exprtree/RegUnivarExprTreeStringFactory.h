#ifndef _REGUNIVAREXPRTREESTRINGFACTORY_H_
#define _REGUNIVAREXPRTREESTRINGFACTORY_H_

#include <helios/adt/exprtree/RegUnivarExprTreeNode.h>
#include <helios/adt/exprtree/UnivarExprTreeStringFactory.h>

#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class extending UnivarExprTreeStringFactory to make register-based
 *  univariate expression trees from expressions given as strings.
 *
 * @tparam NumericType The numeric type of the expression trees to be built by
 *  the factory
 * @see UnivarExprTreeStringFactory
 * @see RegUnivarExprTree
 * @see RegUnivarExprTreeNode
 */
template<typename NumericType,
         typename ExprTreeType = RegUnivarExprTreeNode<NumericType>>
class RegUnivarExprTreeStringFactory
  : public UnivarExprTreeStringFactory<NumericType, ExprTreeType>
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The vector of pointers to the registers
   */
  std::vector<NumericType const*> registers;
  /**
   * @brief The expression register handler for last built tree
   */
  std::shared_ptr<ExpressionRegisterHandler<NumericType>> erh = nullptr;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Construct a string factory to build register-based univariate
   *  expression trees
   * @param registers The vector of pointers to the variables associated to
   *  built trees
   */
  RegUnivarExprTreeStringFactory(
    std::vector<NumericType const*> const& registers)
    : UnivarExprTreeStringFactory<NumericType, ExprTreeType>()
    , registers(registers)
  {
  }
  virtual ~RegUnivarExprTreeStringFactory() = default;

  // ***  MAKE METHODS  *** //
  // ********************** //
  /**
   * @brief Extends the new building process initialization to account for
   *  register related stuff.
   * @see UnivarExprTreeStringFactory::initBuilding
   */
  void initBuilding() override;

  // ***  UTIL METHODS  *** //
  // ********************** //
  /**
   * @brief Extends the UnivarExprTreeStringFactory::handleSymbol method to
   *  support register symbols
   * @see UnivarExprTreeStringFactory::handleSymbol
   */
  void handleSymbol(
    typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol&
      symbol) override;
  /**
   * @brief Extends the UnivarExprTreeStringFactory::newExprTree method to
   *  build register univariate expression trees
   * @param extra It is used to receive the register index
   * @see UnivarExprTreeStringFactory::newExprTree
   */
  ExprTreeType* newExprTree(ExprTreeType* left = nullptr,
                            ExprTreeType* right = nullptr,
                            void* extra = nullptr) override
  {
    return ((extra == nullptr)
              ? new ExprTreeType(erh, 0, left, right)
              : new ExprTreeType(erh, *((size_t*)extra), left, right));
  }
  /**
   * @brief Extends the UnivarExprTreeStringFactory::prepareNextSubExpression
   *  method to handle register-based sub expressions
   * @see UnivarExprTreeStringFactory::prepareNextSubExpression
   */
  void prepareNextSubExpression(
    typename UnivarExprTreeStringFactory<NumericType,
                                         ExprTreeType>::Symbol const& symbol,
    std::string& subexpr) override;
  /**
   * @brief Extends the
   *  UnivarExprTreeStringFactory::extractNamedOrVariableSymbol method to
   *  also extract register symbols by name
   * @see UnivarExprTreeStringFactory::extractNamedOrVariableSymbol
   */
  typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol
  extractNamedOrVariableSymbol(std::string const& symstr) override;
  /**
   * @brief Extends the UnivarExprTreeStringFactory::findEndOfNameIdx method
   *  to deal with register names
   * @see UnivarExprTreeStringFactory::findEndOfNameIdx
   */
  size_t findEndOfNameIdx(std::string const& expr) override;
  /**
   * @brief Count the number of digits required to represent the index of the
   *  register starting the given string.
   *
   * e.g. 1) "ER17" will return 2 because "17" requires 2 digits
   *
   * e.g. 2) "ER17ER5" will return 2 because only the first register is
   *  considered
   *
   * e.g. 3) "ER+pi" will throw an exception because there is no register
   *  index
   *
   * @param symstr String starting by ER and followed by an arbitrary
   *  \f$>0\f$ number of digits specifying the register index
   * @return
   */
  size_t countRegisterIndexDigits(std::string const& symstr);
  /**
   * @brief Extend UnivarExprTreeStringFactory::craftNegSymbol to support
   *  negative expression registers.
   * @see UnivarExprTreeStringFactory::craftNegSymbol
   */
  typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol
  craftNegSymbol(std::string const& expr) override;
};

#include <helios/adt/exprtree/RegUnivarExprTreeStringFactory.tpp>

#endif
