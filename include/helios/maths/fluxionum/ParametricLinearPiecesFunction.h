#pragma once

#include <helios/maths/fluxionum/Function.h>
#include <helios/maths/fluxionum/LinearPiecesFunction.h>

#include <armadillo>

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Parametric linear pieces function
 *
 * \f[
 *  f(x) = \left[\begin{array}{c}
 *      a_{i1}x + b_{i1} \\
 *      \vdots \\
 *      a_{in}x + b_{in}
 *  \end{array}\right]
 * \f]
 *
 * The \f$i\f$-th index comes from \f$S\f$, the sorted set of \f$m\f$ interval
 *  boundaries described for fluxionum::LinearPiecesFunction
 *
 * Notice the domain is not \f$B\f$ but \f$B^n\f$
 *
 * @see fluxionum::Function
 * @see fluxionum::LinearPiecesFunction
 */
template<typename A, typename B>
class ParametricLinearPiecesFunction : public Function<A, arma::Col<B>>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The set \f$S\f$ of \f$m\f$ sorted start points
   * @see fluxionum::LinearPiecesFunction::start
   */
  arma::Col<A> const& start;
  /**
   * @brief The slope for each piece so \f$a_{ij}\f$ is the slope of the
   *  \f$j\f$-th variable at the \f$i\f$-th interval
   */
  arma::Mat<B> const& slope;
  /**
   * @brief The intercept for each piece so \f$b_{ij}\f$ is the intercept of
   *  the \f$j\f$-th variable at the \f$i\f$-th interval
   */
  arma::Mat<B> const& intercept;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief ParametricLinearPiecesFunction default constructor
   * @see fluxionum::ParametricLinearPiecesFunction::start
   * @see fluxionum::ParametricLinearPiecesFunction::slope
   * @see fluxionum::ParametricLinearPiecesFunction::intercept
   */
  ParametricLinearPiecesFunction(arma::Col<A> const& start,
                                 arma::Mat<B> const& slope,
                                 arma::Mat<B> const& intercept)
    : start(start)
    , slope(slope)
    , intercept(intercept)
  {
  }
  virtual ~ParametricLinearPiecesFunction() = default;

  // ***  FUNCTION METHODS  *** //
  // ************************** //
  /**
   * @brief Calculate the image of \f$x\f$ by \f$f\f$ assuming a linear
   *  behavior for each component. In this context, \f$a_{ij}\f$ is the
   *  slope for the \f$j\f$-th variable while \f$b_{ij}\f$ is the intercept
   *  for the \f$j\f$-th variable
   * @param x The input value belonging to the domain of the function
   * @return The image of \f$x\f$ by \f$ft\f$
   */
  arma::Col<B> eval(A const& x) override
  {
    size_t const xIdx = findIndex(x);
    return (x - getStart(xIdx)) * getSlope(xIdx) + getIntercept(xIdx);
  }

  // ***  LINEAR FUNCTION  *** //
  // ************************* //
  /**
   * @see fluxionum::LinearPiecesFunction::getStart
   */
  inline A getStart(size_t const i = 0) const { return start.at(i); }
  /**
   * @brief Obtain the \f$i\f$-th slope vector
   *  \f$a_i = (a_{i1}, \ldots, a_{in})\f$ of the linear function
   * @param i Index of the slope vector to be obtained. In case there are no
   *  multiple slope vectors, the default value \f$i=0\f$ must be used.
   * @return The \f$i\f$-th slope vector
   *  \f$a_i = (a_{i1}, \ldots, a_{in})\f$ of the linear function
   */
  inline arma::Col<B> getSlope(size_t const i = 0) const
  {
    return slope.row(i).as_col();
  }
  /**
   * @brief Obtain the \f$i\f$-th intercept vector
   *  \f$b_i = (b_{i1}, \ldots, b_{in})\f$ of the linear function
   * @param i Index of the intercept vector to be obtained. In case there are
   *  no multiple intercept vectors, the default value \f$i=0\f$ must be
   *  used.
   * @return The \f$i\f$-th intercept vector
   *  \f$b_i = (b_{i1}, \ldots, b_{in})\f$ of the linear function
   */
  inline arma::Col<B> getIntercept(size_t const i = 0) const
  {
    return intercept.row(i).as_col();
  }
  /**
   * @see fluxionum::LinearPiecesFunction::findIndex
   */
  inline size_t findIndex(A const& x) const
  {
    return LinearPiecesFunction<A, B>::findIndex(x, start);
  }
};

}
