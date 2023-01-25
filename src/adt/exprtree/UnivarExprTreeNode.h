#pragma once

#include <IExprTreeNode.h>

#include <cmath>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class implementing a Univariate Expression Tree Node.
 *
 * The Univariate Expression Tree Node supports univariate expressions where
 *  \f$t\f$ is the variable. Univariate expression trees can work with real
 *  numbers (complex are outside its scope). Supported operators are
 *  \f$+, -, *, /, \textrm{^}\f$. Supported constants are \f$e, \pi\f$.
 *  Supported functions are \f$\exp, \ln, \mathrm{sqrt}, \mathrm{abs}, \f$
 *  \f$\cos, \sin, \tan, \mathrm{acos}, \mathrm{asin}, \mathrm{atan}, \f$
 *  \f$\mathrm{atan2}, \cosh, \sinh, \tanh\f$. However since \f$atan2\f$ is a
 *  bivariate function, it is internally considered as an operator due to an
 *  implementation trick.
 *
 * @tparam NumericType The numeric type to be used by the univariate
 *  expression tree. It must be contained or equal to the reals (
 *  \f$\subseteq \mathbb{R}\f$).
 */
template <typename NumericType>
class UnivarExprTreeNode : public IExprTreeNode<NumericType, NumericType> {
public:
    // ***   E N U M   *** //
    // ******************* //
    /**
     * @brief The different types of element (symbols) supported by the
     *  univariate expression tree node
     */
    enum SymbolType {OPERATOR, NUMBER, VARIABLE, FUNCTION};
    /**
     * @brief The different operators supported by the univariate expression
     *  tree node
     */
    enum OpType {
        OP_ADD,     // + Addition
        OP_SUB,     // - Subtraction
        OP_MUL,     // * Product
        OP_DIV,     // / Division
        OP_POW,     // ^ Power
        OP_IPOW,    // ^ Iterative Power (smaller exponents could lead to
                    // faster computation but they MUST be integer > 0)
        OP_atan2    // atan2 (computationally convenient implementation)
    };
    /**
     * @brief The different functions supported by the univariate expression
     *  tree node
     */
    enum FunType {
        f_exp,      // Exponential Function
        f_ln,       // Napierian Logarithm
        f_sqrt,     // Square Root
        f_abs,      // Absolute Value
        f_cos,      // Cosine
        f_sin,      // Sine
        f_tan,      // Tangent
        f_acos,     // Arccosine
        f_asin,     // Arcsine
        f_atan,     // Arctangent
        f_cosh,     // Hyperbolic Cosine
        f_sinh,     // Hyperbolic Sine
        f_tanh      // Hyperbolic Tangent
    };

    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The type of element (symbol) used by the univariate expression
     *  tree node
     * @see UnivarExprTreeNode::SymbolType
     */
    SymbolType symbolType;
    /**
     * @brief The element of the node
     */
    union {
        /**
         * @brief Number as node's element
         */
        NumericType num;
        /**
         * @brief Operator as node's element
         */
        OpType op;
        /**
         * @brief Function as node's element
         */
        FunType fun;
    };
    /**
     * @brief The left child node
     */
    UnivarExprTreeNode *left;
    /**
     * @brief The right child node
     */
    UnivarExprTreeNode *right;


    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for Univariate Expression Tree Node
     * @param left The left child node
     * @param right The right child node
     */
    UnivarExprTreeNode(
        UnivarExprTreeNode *left=nullptr, UnivarExprTreeNode *right=nullptr
    ) : left(left), right(right) {}

    virtual ~UnivarExprTreeNode(){
        if(left != nullptr){
            delete left;
            left = nullptr;
        }
        if(right != nullptr){
            delete right;
            right = nullptr;
        }
    }

    // ***  UNIVARIATE EXPRESSION TREE NODE METHODS  *** //
    // ************************************************* //
    /**
     * @brief Do the operation if the node is an operator node. Otherwise,
     *  something really wrong might happen. Something that you don't want to
     *  experiment.
     * @param x The left hand side (lhs) \f$x\f$
     * @param y The right hand side (rhs) \f$x\f$
     * @return Result of applying the operator \f$\odot\f$ such that
     *  \f$x \odot y\f$
     */
    inline NumericType doOperation(
        NumericType const x, NumericType const y
    ) const {
        switch(op){
            case OP_ADD:    return x + y;
            case OP_SUB:    return x - y;
            case OP_MUL:    return x * y;
            case OP_DIV:    return x / y;
            case OP_POW:    return std::pow(x, y);
            case OP_IPOW: {
                NumericType out = x;
                int const n = (int) y;
                for (int i = 1; i < n; ++i) out *= x;
                return out;
            }
            case OP_atan2:  return std::atan2(x, y);
        }
        std::stringstream ss;
        ss  << "UnivarExprTreeNode failed to do operation with "
            << "x = " << x << ", y = " << y;
        throw HeliosException(ss.str());
    }
    /**
     * @brief Compute the function's output for the given input \f$x\f$
     * @param x The function's input \f$x\f$
     * @return The function's output \f$f(x)\f$
     */
    inline NumericType doFunction(
        NumericType const x
    ) const {
        switch(fun){
            case f_exp:     return std::exp(x);
            case f_ln:      return std::log(x);
            case f_sqrt:    return std::sqrt(x);
            case f_abs:     return std::abs(x);
            case f_cos:     return std::cos(x);
            case f_sin:     return std::sin(x);
            case f_tan:     return std::tan(x);
            case f_acos:    return std::acos(x);
            case f_asin:    return std::asin(x);
            case f_atan:    return std::atan(x);
            case f_cosh:    return std::cosh(x);
            case f_sinh:    return std::sinh(x);
            case f_tanh:    return std::tanh(x);
        }
        std::stringstream ss;
        ss  << "UnivarExprTreeNode failed to do function with "
            << "x = " << x;
        throw HeliosException(ss.str());
    }

    // ***  EXPRESSION TREE NODE INTERFACE *** //
    // *************************************** //
    /**
     * @see IExprTreeNode::eval
     */
    NumericType eval(NumericType const t) const override{
        switch(symbolType){
            case OPERATOR:
                return doOperation(left->eval(t), right->eval(t));
            case NUMBER:
                return num;
            case VARIABLE:
                return t;
            case FUNCTION:
                return doFunction(left->eval(t));
        }
        std::stringstream ss;
        ss  << "UnivarExprTreeNode failed to eval t = " << t;
        throw HeliosException(ss.str());
    }

    // ***  BINARY TREE INTERFACE  *** //
    // ******************************* //
    /**
     * @brief Obtain the left child of current node
     * @return Left child
     */
    IBinaryTreeNode * getLeftChild() const override {return left;}
    /**
     * @brief Obtain the right child of current node
     * @return Right child
     */
    IBinaryTreeNode * getRightChild() const override {return right;}
    /**
     * @brief Simplified check because for UnivarExprTreeNode there is no right
     *  child without a left child.
     * @see IBinaryTreeNode::isLeafNode
     */
    bool isLeafNode() const override {return getLeftChild()==nullptr;}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Check whether the node is an operator (true) or not (false)
     * @return True if node is an operator, false otherwise
     */
    inline bool isOperator() const {return symbolType == OPERATOR;}
    /**
     * @brief Check whether the node is a function (true) or not (false)
     * @return True if node is a function, false otherwise
     */
    inline bool isFunction() const {return symbolType == FUNCTION;}
    /**
     * @brief Check whether the node is a number (true) or not (false)
     * @return True if node is a number, false otherwise
     */
    inline bool isNumber() const {return symbolType == NUMBER;}
    /**
     * @brief Check whether the node is a variable (true) or not (false)
     * @return True if node is a variable, false otherwise
     */
    inline bool isVariable() const {return symbolType == VARIABLE;}
    /**
     * @brief Set the operator UnivarExprTreeNode::op from given string
     * @param opStr String representing an operator for the node
     */
    inline void setOperator(std::string const &opStr){
        if(opStr == "+") op = OP_ADD;
        else if(opStr == "-") op = OP_SUB;
        else if(opStr == "*") op = OP_MUL;
        else if(opStr == "/") op = OP_DIV;
        else if(opStr == "^") op = OP_POW;
        else if(opStr == "atan2") op = OP_atan2;
        else {
            std::stringstream ss;
            ss << "UnivarExprTreeNode::setOperator(std::string const &) "
               << "failed due to an unexpected operator: \"" << opStr << "\"";
            throw HeliosException(ss.str());
        }
    }
};