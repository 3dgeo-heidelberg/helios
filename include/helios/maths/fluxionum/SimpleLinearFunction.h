#pragma once

#include <helios/maths/fluxionum/Function.h>

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Simple linear function
 *
 * \f[
 *  f(x) = ax + b
 * \f]
 *
 * @see fluxionum::Function
 */
template<typename A, typename B>
class SimpleLinearFunction : public Function<A, B>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The slope \f$a\f$ of the linear function
   */
  B slope;
  /**
   * @brief The intercept \f$b\f$ of the linear function
   */
  B intercept;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief SimpleLinearFunction default constructor
   * @see fluxionum::SimpleLinearFunction::slope
   * @see fluxionum::SimpleLinearFunction::intercept
   */
  SimpleLinearFunction(B const slope, B const intercept)
    : Function<A, B>()
    , slope(slope)
    , intercept(intercept)
  {
  }
  virtual ~SimpleLinearFunction() = default;

  // ***  FUNCTION METHODS  *** //
  // ************************** //
  /**
   * @brief Calculate the image of \f$x\f$ by \f$f\f$ assuming a linear
   *  behavior where \f$a\f$ is the slope and \f$b\f$ is the intercept.
   *
   * \f[
   *  f(x) = ax + b
   * \f]
   *
   * @param x The input value belonging to the domain of the function
   * @return The image of \f$x\f$ by \f$f\f$
   */
  B eval(A const& x) override { return x * slope + intercept; }

  // ***  LINEAR FUNCTION  *** //
  // ************************* //
  /**
   * @brief Obtain the slope \f$a\f$ of the linear function
   * @return The slope \f$a\f$ of the linear function
   */
  inline B getSlope() const { return slope; }
  /**
   * @brief Obtain the intercept \f$b\f$ of the linear function
   * @return The intercept \f$b\f$ of the linear function
   */
  inline B getIntercept() const { return intercept; }
};

}
