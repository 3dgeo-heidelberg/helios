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
};
}}

#include <maths/Scalar.tpp>

#endif
