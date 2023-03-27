#include <RegUnivarExprTreeStringFactory.h>

// ***  MAKE METHODS  *** //
// ********************** //
template <typename NumericType, typename ExprTreeType>
void RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::initBuilding(){
    // Call building initialization from parent
    UnivarExprTreeStringFactory<NumericType, ExprTreeType>::initBuilding();
    // Built expression register handler for current tree
    erh = std::make_shared<ExpressionRegisterHandler<NumericType>>(registers);
}

// ***  UTIL METHODS  *** //
// ********************** //
template <typename NumericType, typename ExprTreeType>
void RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::handleSymbol(
    struct UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
        Symbol &symbol
){
    // Handle register symbols here
    if(symbol.type==UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION){
        size_t regIdx = (size_t) std::stoi(symbol.str);
        ExprTreeType *node = newExprTree(nullptr, nullptr, &regIdx);
        node->symbolType = symbol.type;
        UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
            nodes.push_back(node);
        return;
    }
    // Delegate non-register stuff to parent UnivarExprTreeStringFactory
    UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
        handleSymbol(symbol);
}

template <typename NumericType, typename ExprTreeType>
void RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
prepareNextSubExpression(
    struct UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
        Symbol const &symbol,
    std::string &subexpr
){
    // TODO Rethink : Is this method necessary? If not, dont make base virtual
    // Handle register names here
    if(symbol.type==UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION){
        size_t const regStrLen = 2+countRegisterIndexDigits(subexpr);
        subexpr = subexpr.substr(regStrLen);
        return;
    }
    // Delegate non-register stuff to parent UnivarExprTreeStringFactory
    UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
        prepareNextSubExpression(symbol, subexpr);
}

template <typename NumericType, typename ExprTreeType>
struct UnivarExprTreeStringFactory<NumericType, ExprTreeType>::Symbol
RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
extractNamedOrVariableSymbol(
    std::string const &symstr
) {
    // Extract expression register symbol
    if(symstr.substr(0, 2) == "ER"){
        // Extract register index
        size_t const numConsecutiveDigits = countRegisterIndexDigits(symstr);
        if(numConsecutiveDigits < 1){
            std::stringstream ss;
            ss  << "RegUnivarExprTreeStringFactory::"
                   "extractNamedOrVariableSymbol "
                   "failed because a register with no index was found in "
                   "symbol string: \"" << symstr << "\"";
            throw HeliosException(ss.str());
        }
        std::string regIdxStr = symstr.substr(2, numConsecutiveDigits);
        // Build register symbol (using EXTENSION symbol type)
        struct UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
            Symbol symbol;
        symbol.type = UnivarExprTreeNode<NumericType>::SymbolType::EXTENSION;
        symbol.str = regIdxStr;
        UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
            lastReadIsOpenPriorityOrSeparator = false;
        return symbol;
    }
    // Delegate upon UnivarExprTreeStringFactory extraction method
    return
        UnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
            extractNamedOrVariableSymbol(symstr);
}

template <typename NumericType, typename ExprTreeType>
size_t RegUnivarExprTreeStringFactory<NumericType, ExprTreeType>::\
countRegisterIndexDigits(
    std::string const &symstr
){
    size_t const numChars = symstr.size();
    size_t numConsecutiveDigits = 0;
    for(size_t i = 2 ; i < numChars ; ++i){ // Iterate string after ER
        if(std::isdigit(symstr[i])) ++numConsecutiveDigits;
        else break;
    }
    return numConsecutiveDigits;
}
