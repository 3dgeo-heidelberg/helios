#pragma once

#include <fluxionum/Function.h>

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Iterative Euler method
 *
 * \f[
 *  y(t+h) = y(t) + h \frac{dy}{dt}(t)
 * \f]
 *
 * The iterative Euler method computes the Euler method assuming at each
 *  iteration the given step. It has an error order \f$Ch\f$ where \f$h\f$
 *  is the step size.
 *
 *
 * @see fluxionum::Function
 * @see fluxionum::ParametricIterativeEulerMethod
 */
template<typename A, typename B>
class IterativeEulerMethod : public Function<A, B>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Reference to the derivative function
   * \f[
   *  \frac{dy}{dt}(t)
   * \f]
   * @see fluxionum::IterativeEulerMethod::y
   * @see fluxionum::IterativeEulerMethod::t
   */
  Function<A, B>& dydt;
  /**
   * @brief The initial value of \f$t\f$, \f$t_0\f$
   * @see fluxionum::IterativeEulerMythod::y0
   */
  A t0;
  /**
   * @brief The current value of \f$t\f$
   * @see fluxionum::IterativeEulerMethod::y
   * @see fluxionum::IterativeEulerMethod::dydt
   */
  A t;
  /**
   * @brief The initial value of \f$y\f$, \f$y(t_0)\f$
   * @see fluxionum::IterativeEulerMethod::t0
   */
  B y0;
  /**
   * @brief The current value of \f$y(t)\f$
   * @see fluxionum::IterativeEulerMethod::t
   * @see fluxionum::IterativeEulerMethod::dydt
   */
  B y;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief IterativeEulerMethod default constructor
   * @see fluxionum::IterativeEulerMethod::dydt
   * @see fluxionum::IterativeEulerMethod::t0
   * @see fluxionum::IterativeEulerMethod::y0
   */
  IterativeEulerMethod(Function<A, B>& dydt, A const& t0, B const& y0)
    : Function<A, B>()
    , dydt(dydt)
    , t0(t0)
    , t(t0)
    , y0(y0)
    , y(y0)
  {
  }
  virtual ~IterativeEulerMethod() = default;

  // ***  FUNCTION METHODS  *** //
  // ************************** //
  /**
   * @brief Iteratively compute the next value using Euler method.
   *
   * It is assumed that the instance knows the current time, the current
   *  value and the current derivative. Thus, it can approximate the next
   *  value.
   *
   * Calling this method updates the internal status of the
   *  IterativeEulerMethod so it is representing the new state at \f$t+h\f$.
   *
   * @param h The step size
   * @return The computed value
   */
  B eval(A const& h) override
  {
    y = y + h * dydt(t);
    t = t + h;
    return y;
  }

  /**
   * @brief Restart the IterativeEulerMethod so it is at its
   *  initial state again \f$y(t_0) = y_0\f$
   */
  virtual void restart()
  {
    setT(getT0());
    setY(getY0());
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @see fluxionum::IterativeEulerMethod::dydt
   */
  inline Function<A, B> const& getDydt() const { return dydt; };
  /**
   * @see fluxionum::IterativeEulerMethod::t
   */
  inline A getT() const { return t; };
  /**
   * @see fluxionum::IterativeEulerMethod::t
   */
  inline void setT(A const t) { this->t = t; }
  /**
   * @see fluxionum::IterativeEulerMethod::t0
   */
  inline A getT0() const { return t0; }
  /**
   * @see fluxionum::IterativeEulerMethod::y
   */
  inline B getY() const { return y; }
  /**
   * @see fluxionum::IterativeEulerMethod::y
   */
  inline void setY(B const y) { this->y = y; }
  /**
   * @see fluxionum::IterativeEulerMethod::y0
   */
  inline B getY0() const { return y0; }
};

}
