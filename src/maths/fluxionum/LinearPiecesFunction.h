#pragma once

#include <fluxionum/Function.h>

#include <armadillo>

namespace fluxionum{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Linear pieces function
 *
 * \f[
 *  f(x) = a_{i}(x-s_i) + b_{i}
 * \f]
 *
 * There is a sorted set of \f$m\f$ interval boundaries
 *  \f$S=\{s_1, \ldots, s_m\} \subset A\f$. Thus, the index \f$i\f$ to
 *  determine the slope \f$a_i\f$ and the intercept \f$b_i\f$ can be obtained
 *  as \f$x \in  [s_i, s_{i+1})\f$ for the general case. For the particular
 *  case of \f$i=m\f$ it is \f$x \in [s_{m}, \infty)\f$
 *
 * @see fluxionum::Function
 */
template <typename A, typename B>
class LinearPiecesFunction : public Function<A, B>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The set \f$S\f$ of \f$m\f$ sorted start points
     * @see fluxionum::ParametricLinearPiecesFunction::start
     */
    arma::Col<A> const &start;
    /**
     * @brief The slopes for each piece \f$a_1, \ldots, a_m\f$
     */
    arma::Col<B> const &slope;
    /**
     * @brief The intercepts for each piece \f$b_1, \ldots, b_m\f$
     */
    arma::Col<B> const &intercept;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief LinearPiecesFunction default constructor
     * @see fluxionum::LinearPiecesFunction::start
     * @see fluxionum::LinearPiecesFunction::slope
     * @see fluxionum::LinearPiecesFunction::intercept
     */
    LinearPiecesFunction(
        Col<A> const &start,
        Col<B> const &slope,
        Col<B> const &intercept
    ) :
        start(start),
        slope(slope),
        intercept(intercept)
    {}
    virtual ~LinearPiecesFunction() = default;

    // ***  FUNCTION METHODS  *** //
    // ************************** //
    /**
     * @brief Calculate the image of \f$x\f$ by \f$f\f$ assuming a linear
     *  behavior where \f$a_i\f$ is the slope and \f$b_i\f$ is the intercept.
     *
     * \f[
     *  f(x) = a_i x + b_i
     * \f]
     *
     * @param x The input value belonging to the domain of the function
     * @return The image of \f$x\f$ by \f$f\f$
     */
    B eval(A const &x) override{
        size_t const xIdx = findIndex(x);
        return (x-getStart(xIdx))*getSlope(xIdx) + getIntercept(xIdx);
    }


    // ***  LINEAR FUNCTION  *** //
    // ************************* //
    /**
     * @brief Obtain the \f$i\f$-th start point \f$s_i\f$ of the linear
     *  function
     * @param i Index of the start point to be obtained,
     * @return The \f$i\f$-th start point of the linear function
     */
    inline A getStart(size_t const i=0) const {return start.at(i);}
    /**
     * @brief Obtain the \f$i\f$-th slope \f$a_i\f$ of the linear function
     * @param i Index of the slope to be obtained. In case there are no
     *  multiple slopes, the default value \f$i=0\f$ must be used.
     * @return The \f$i\f$-th slope \f$a_i\f$ of the linear function
     */
    inline B getSlope(size_t const i=0) const {return slope.at(i);}
    /**
     * @brief Obtain the \f$i\f$-th intercept \f$b_i\f$ of the linear function
     * @param i Index of the intercept to be obtained. In case there are no
     *  multiple intercepts, the default value \f$i=0\f$ must be used.
     * @return The \f$i\f$-th intercept \f$b_i\f$ of the linear function
     */
    inline B getIntercept(size_t const i=0) const {return intercept.at(i);}
    /**
     * @brief Obtain the index identifying the interval where \f$x\f$ belongs
     *  to. It is, find \f$i\f$ such that \f$x \in [s_{i}, s_{i+1})\f$
     * @param x The input value belonging to the domain of the function
     * @return The index identifying the interval where \f$x\f$ belongs to
     * @see LinearPiecesFunction::findIndex(A const &, arma::Col<A> const &)
     */
    inline size_t findIndex(A const &x) const{return findIndex(x, start);}
    /**
     * @brief Assist the LinearPiecesFunction::findIndex(A const &)
     * @see fluxionum::LinearPiecesFunction::findIndex(A const &)
     */
    static inline size_t findIndex(
        A const &x,
        arma::Col<A> const &start
    ) {
        size_t const m = start.n_elem-1;
        for(size_t i = m ; i > 0 ; --i) if(x >= start.at(i)) return i;
        return 0;
    }
};

}