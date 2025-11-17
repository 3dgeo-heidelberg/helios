#include <adt/exprtree/UnivarExprTreeStringFactory.h>
#include <assetloading/XmlUtils.h>

// ***  STATIC METHODS  *** //
// ************************ //
template<typename NumericType>
std::shared_ptr<UnivarExprTreeNode<NumericType>>
XmlUtils::createUnivarExprTree(tinyxml2::XMLElement* exprNode)
{
  // Retrieve expression
  std::string expr = exprNode->FindAttribute("expr")->Value();

  // Build univariate expression tree
  UnivarExprTreeStringFactory<NumericType> uetsf;
  return std::static_pointer_cast<UnivarExprTreeNode<NumericType>>(
    uetsf.makeShared(expr));
}

template<typename NumericType>
std::shared_ptr<UnivarExprTreeNode<NumericType>>
XmlUtils::createUnivarExprTree(
  tinyxml2::XMLElement* exprNode,
  std::unordered_map<std::string, std::string> const& renameMap)
{
  // Retrieve expression
  std::string expr = exprNode->FindAttribute("expr")->Value();

  // Do rename
  std::unordered_map<std::string, std::string>::const_iterator it;
  for (it = renameMap.begin(); it != renameMap.end(); ++it) {
    std::string const user = it->first;  // User-friendly name
    std::string const tree = it->second; // Tree-friendly name
    std::size_t exprIdx = expr.find(user);
    while (exprIdx != std::string::npos) {
      std::string left = expr.substr(0, exprIdx);
      std::string const right = expr.substr(exprIdx + user.size());
      expr = left.append(tree).append(right);
      exprIdx = expr.find(user);
    }
  }

  // Build univariate expression tree
  UnivarExprTreeStringFactory<NumericType> uetsf;
  return std::static_pointer_cast<UnivarExprTreeNode<NumericType>>(
    uetsf.makeShared(expr));
}
