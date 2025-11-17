#include <RegUnivarExprTreeStringFactory.h>

// ***  MAKE METHODS  *** //
// ********************** //
template<typename NumericType, typename ExprTreeType>
void
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::initBuilding()
{
  // Call building initialization from parent
  UnivarExprTreeStringFactory<NumericType, ExprTreeType>::initBuilding();
  // Built expression register handler for current tree
  erh = std::make_shared<ExpressionRegisterHandler<NumericType>>(registers);
}

// ***  UTIL METHODS  *** //
// ********************** //
template<typename NumericType, typename ExprTreeType>
void
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::handleSymbol(
  typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol&
    symbol)
{
  // Handle register symbols here
  if (symbol.type == UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION) {
    if (symbol.str[0] == '-') { // Handle negative sign
      size_t regIdx = (size_t)std::stoi(symbol.str.substr(1));
      ExprTreeType* right = newExprTree(nullptr, nullptr, &regIdx);
      right->symbolType = symbol.type;
      ExprTreeType* left = newExprTree();
      left->symbolType = UnivarExprTreeNode<NumericType>::SymbolType::NUMBER;
      left->num = -1.0;
      ExprTreeType* node = newExprTree(left, right);
      node->symbolType = UnivarExprTreeNode<NumericType>::SymbolType::OPERATOR;
      node->op = UnivarExprTreeNode<NumericType>::OpType::OP_MUL;
      UnivarExprTreeStringFactory<NumericType, ExprTreeType>::nodes.push_back(
        node);
    } else { // Handle no sign
      size_t regIdx = (size_t)std::stoi(symbol.str);
      ExprTreeType* node = newExprTree(nullptr, nullptr, &regIdx);
      node->symbolType = symbol.type;
      UnivarExprTreeStringFactory<NumericType, ExprTreeType>::nodes.push_back(
        node);
    }
    return;
  }
  // Delegate non-register stuff to parent UnivarExprTreeStringFactory
  UnivarExprTreeStringFactory<NumericType, ExprTreeType>::handleSymbol(symbol);
}

template<typename NumericType, typename ExprTreeType>
void
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::
  prepareNextSubExpression(
    typename UnivarExprTreeStringFactory<NumericType,
                                         ExprTreeType>::Symbol const& symbol,
    std::string& subexpr)
{
  // Handle register names here
  if (symbol.type == UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION) {
    if (symbol.str[0] == '-') { // Handle for negative expression registers
      size_t const regStrLen = 3 + countRegisterIndexDigits(subexpr.substr(1));
      subexpr = subexpr.substr(regStrLen);
    } else { // Handle for non-negative expression registers
      size_t const regStrLen = 2 + countRegisterIndexDigits(subexpr);
      subexpr = subexpr.substr(regStrLen);
    }
    return;
  }
  // Delegate non-register stuff to parent UnivarExprTreeStringFactory
  UnivarExprTreeStringFactory<NumericType,
                              ExprTreeType>::prepareNextSubExpression(symbol,
                                                                      subexpr);
}

template<typename NumericType, typename ExprTreeType>
typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::
  extractNamedOrVariableSymbol(std::string const& symstr)
{
  // Extract expression register symbol
  if (symstr.substr(0, 2) == "ER") {
    // Extract register index
    size_t const numConsecutiveDigits = countRegisterIndexDigits(symstr);
    if (numConsecutiveDigits < 1) {
      std::stringstream ss;
      ss << "RegUnivarExprTreeStringFactory::"
            "extractNamedOrVariableSymbol "
            "failed because a register with no index was found in "
            "symbol string: \""
         << symstr << "\"";
      throw HeliosException(ss.str());
    }
    std::string regIdxStr = symstr.substr(2, numConsecutiveDigits);
    // Build register symbol (using EXTENSION symbol type)
    typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol
      symbol;
    symbol.type = UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION;
    symbol.str = regIdxStr;
    UnivarExprTreeStringFactory<NumericType, ExprTreeType>::
      lastReadIsOpenPriorityOrSeparator = false;
    return symbol;
  }
  // Delegate upon UnivarExprTreeStringFactory extraction method
  return UnivarExprTreeStringFactory<NumericType, ExprTreeType>::
    extractNamedOrVariableSymbol(symstr);
}

template<typename NumericType, typename ExprTreeType>
size_t
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::findEndOfNameIdx(
  std::string const& expr)
{
  // Call parent find method
  size_t nonNameIdx =
    UnivarExprTreeStringFactory<NumericType, ExprTreeType>::findEndOfNameIdx(
      expr);
  // Find end of register name (if expression starts with a register name)
  size_t const m = expr.size(); // Num. characters in expression
  std::string symstr = expr.substr(0, nonNameIdx);
  if (symstr == "ER") {
    for (size_t i = 2; i < m; ++i) {
      if (std::isdigit(expr[i]))
        nonNameIdx = i + 1;
      else
        break;
    }
  }
  // Return index of first non-name character in expression
  return nonNameIdx;
}

template<typename NumericType, typename ExprTreeType>
size_t
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::
  countRegisterIndexDigits(std::string const& symstr)
{
  size_t const numChars = symstr.size();
  size_t numConsecutiveDigits = 0;
  for (size_t i = 2; i < numChars; ++i) { // Iterate string after ER
    if (std::isdigit(symstr[i]))
      ++numConsecutiveDigits;
    else
      break;
  }
  return numConsecutiveDigits;
}

template<typename NumericType, typename ExprTreeType>
typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::craftNegSymbol(
  std::string const& expr)
{
  // Handle crafting of negative expression registers
  if (expr[1] == 'E' && expr[2] == 'R') {
    typename UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol
      symbol;
    symbol.type = UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION;
    size_t const numRegIdxDigits = countRegisterIndexDigits(expr.substr(1));
    symbol.str = "-" + expr.substr(3, 3 + numRegIdxDigits);
    return symbol;
  }
  // If it is not a negative expression register, delegate upon parent
  return UnivarExprTreeStringFactory<NumericType, ExprTreeType>::craftNegSymbol(
    expr);
}
