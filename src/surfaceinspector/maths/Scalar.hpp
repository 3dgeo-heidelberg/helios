#ifndef _SURFACEINSPECTOR_MATHS_SCALAR_HPP_
#define _SURFACEINSPECTOR_MATHS_SCALAR_HPP_

#include <surfaceinspector/util/Object.hpp>

using SurfaceInspector::util::Object;

namespace SurfaceInspector { namespace maths {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing common operations to work with scalars
 * @tparam T Type of scalar to work with
 */
template <typename T>
class Scalar : public Object{
public:
    // ***  GENERAL PURPOSE  *** //
    // ************************* //
    /**
     * @brief Compute the factorial of \f$n\f$ (\f$n!\f$)
     * @param n Integer number which factorial must be computed
     * @return Factorial of \f$n\f$ (\f$n!\f$)
     */
    static T factorial(T const n);
    /**
     * @brief Compute Newton binomial \f${{n}\choose{k}}\f$
     * @return Newton binomial \f${{n}\choose{k}}\f$
     */
    static T binom(T const n, T const k);

    // ***  COMBINATORY  *** //
    // ********************* //
    /**
     * @brief Compute combinations (order does not matter) of \$fn\f$ elements
     *  considering \f$k\f$ with no repetitions
     *
     * \f[
     *  {{n}\choose{k}}
     * \f]
     */
    static T inline combinationsNoRepetition(T const n, T const k)
    {return binom(n, k);}
    /**
     * @brief Compute combinations (order does not matter) of \f$n\f$ elements
     *  considering \f$k\f$ allowing repetitions
     *
     * \f[
     *  {{n+k-1}\choose{k}}
     * \f]
     */
    static T inline combinationsRepetition(T const n, T const k)
    {return binom(n+k-1, k);}
    /**
     * @brief Compute variations (order does matter) of \f$n\f$ elements
     *  considering \f$k\f$ with no repetitions
     *
     * \f[
     *  \frac{n!}{(n-k)!}
     * \f]
     */
    static T variationsNoRepetition(T const n, T const k);
    /**
     * @brief Compute variations (order does matter) of \f$n\f$ elements
     *  considering \f$k\f$ with repetitions
     *
     * \f[
     *  n^{k}
     * \f]
     */
    static T inline variationsRepetition(T const n, T const k);
    /**
     * @brief Compute the base \f$2\f$ power \f$2^k\f$ in a fast way. Notice
     *  this method only works with \f$k \geq 0\f$ exponents and integer
     *  numerical types.
     *
     * If a power has base \f$2\f$, then it can be computed exploiting bit
     *  shift operations as illustrated below:
     *
     * \f[
     * \left\{\begin{array}{lllll}
     *  2^0 &=& 1 &=& 1 \ll 0 \\
     *  2^1 &=& 2 &=& 1 \ll 1 \\
     *  2^2 &=& 4 &=& 1 \ll 2 \\
     *  2^3 &=& 8 &=& 1 \ll 3 \\
     *  \vdots &\vdots& \vdots &\vdots& \vdots \\
     *  2^k &=& \prod_{i=1}^{k}2 &=& 1 \ll k
     * \end{array}\right.
     * \f]
     *
     * The bit shift operation \f$\ll k\f$ means shifting the binary
     *  representation of the number to the left, padding with \f$0\f$. Thus,
     *  the number \f$4 = 0100_2\f$ could be easily obtained from number
     *  \f$1 = 0001_2\f$ simply by \f$1 \ll 2 = 0001_2 \ll 2 = 0100_2 = 2\f$.
     *
     * @param k The \f$k \geq 0\f$ integer exponent
     * @return Computed power \f$2^k \in \mathbb{Z}_{\geq 0}\f$
     */
    static T inline pow2(T const k)
    {return ((T)1) << k;}
};
}}

#include <surfaceinspector/maths/Scalar.tpp>

#endif
