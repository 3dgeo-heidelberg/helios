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
 *  \f$+, -, *, /, \^|f$. Supported constants are $e, \pi$. Supported functions
 *  are \f$\exp, \ln, \sqrt, \cos, \sin, \tan, \operatorname{atan}, \f$
 *  \f$\operatorname{abs}\f$.
 *
 * @tparam NumericType The numeric type to be used by the univariate
 *  expression tree. It must be contained or equal to the reals (
 *  \f$\subset \mathbb{R}\f$).
 */
template <typename NumericType>
class UnivarExprTreeNode : public IExprTreeNode<NumericType> {
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
        f_sqrt,     // Square root
        f_cos,      // Cosine
        f_sin,      // Sine
        f_tan,      // Tangent
        f_acos,     // Arccosine
        f_asin,     // Arcsine
        f_atan      // Arctangent
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

    virtual ~UnivarExprTreeNode() = default;

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
    inline NumericType doOperation(NumericType const x, NumericType const y){
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
    }
    /**
     * @brief Compute the function's output for the given input \f$x\f$
     * @param x The function's input \f$x\f$
     * @return The function's output \f$f(x)\f$
     */
    inline NumericType doFunction(NumericType const x){
        switch(fun){
            case f_exp:     return std::exp(x);
            case f_ln:      return std::log(x);
            case f_sqrt:    return std::sqrt(x);
            case f_cos:     return std::cos(x);
            case f_sin:     return std::sin(x);
            case f_tan:     return std::tan(x);
            case f_acos:    return std::acos(x);
            case f_asin:    return std::asin(x);
            case f_atan:    return std::atan(x);
        }
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
};