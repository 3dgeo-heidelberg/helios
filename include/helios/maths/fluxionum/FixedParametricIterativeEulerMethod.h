#pragma once

#include <helios/maths/fluxionum/ParametricIterativeEulerMethod.h>

#include <armadillo>

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Fixed parametric iterative Euler method
 *
 * It is like the ParametricIterativeEulerMethod, but the values at the
 *  frontiers are given and thus can be used to fix/correct the function value
 *  at those points. This helps preserving the stability of the approximation,
 *  specially for those cases which have a significant number of frontiers
 *
 * @see fluxionum::ParametricIterativeEulerMethod
 * @see fluxionum::FixedIterativeEulerMethod
 * @see fluxionum::Function
 */
template<typename A, typename B>
class FixedParametricIterativeEulerMethod
  : public ParametricIterativeEulerMethod<A, B>
{
protected:
  // ***  USING  *** //
  // *************** //
  using ParametricIterativeEulerMethod<A, B>::t;
  using ParametricIterativeEulerMethod<A, B>::y;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The \f$m\f$ frontiers such that
   *  \f$\forall t,\, \exists i \ni t \in [a_i, a_{i+1})\f$.
   * For the last frontier, the interval is \f$[a_{m}, \infty)\f$.
   */
  arma::Col<A> const& ta;
  /**
   * @brief The value of \f$\vec{y}(t_i)\f$ at each of the \f$m\f$ frontiers
   *  such that
   *  \f$\forall i,\, \vec{y_i} = \left(y_{i1}, \ldots, y_{in}\right)\f$
   */
  arma::Mat<B> const& ya;
  /**
   * @brief The index of the current piece
   *
   * \f[
   *  i : t_{i} \in [a_{i}, a_{i+1})
   * \f]
   */
  std::size_t i;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief FixedParametricIterativeEulerMethod default constructor
   * @param i The index of the initial piece for the iterative method. By
   *  default, it is assumed to be 0, it is the first piece.
   * @see ParametricIterativeEulerMethod::ParametricIterativeEulerMethod
   * @see FixedParametricIterativeEulerMethod::ta
   * @see FixedParametricIterativeEulerMethod::ya
   * @see FixedParametricIterativeEulerMethod::i
   */
  FixedParametricIterativeEulerMethod(Function<A, arma::Col<B>>& dydt,
                                      A const& t0,
                                      arma::Col<B> const& y0,
                                      arma::Col<A> const& ta,
                                      arma::Mat<B> const& ya,
                                      std::size_t const i = 0)
    : ParametricIterativeEulerMethod<A, B>(dydt, t0, y0)
    , ta(ta)
    , ya(ya)
    , i(i)
  {
  }
  virtual ~FixedParametricIterativeEulerMethod() = default;

  // ***  FUNCTION METHODS  *** //
  // ************************** //
  /**
   * @brief Compute the parametric iterative Euler method but considering
   *  given fixed frontiers
   * @param h The step between current \f$t_i\f$ and next \f$t_{i+1}\f$ such
   *  that \f$h = t_{i+1} - t_i\f$
   * @see fluxionum::ParametricIterativeEulerMethod::eval
   */
  arma::Col<B> eval(A const& h) override
  {
    // Handle fixed frontiers (assuming time is sorted)
    A th = t + h;
    A tdiff = h;
    if ((i < ta.n_elem - 1) && (th >= ta.at(i + 1))) { // Update frontier
      for (size_t j = ta.n_elem - 1; j > i; --j) {
        if (th >= ta.at(j)) {
          tdiff = h - (ta.at(j) - t);
          t = ta.at(j);
          y = ya.row(j).as_col();
          i = j;
          break;
        }
      }
    }
    // Compute iterative Euler method
    return ParametricIterativeEulerMethod<A, B>::eval(tdiff);
  }

  /**
   * @see ParametricIterativeEulerMethod::restart
   */
  void restart() override
  {
    ParametricIterativeEulerMethod<A, B>::restart();
    setCurrentPieceIndex(0);
  }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the index identifying the current piece
   * @see fluxionum::FixedParametricIterativeEulerMethod::i
   */
  inline std::size_t getCurrentPieceIndex() const { return i; }
  /**
   * @brief Set the index identifying the current piece
   * @see fluxionum::FixedParametricIterativeEulerMethod::i
   */
  inline void setCurrentPieceIndex(std::size_t const i) { this->i = i; }
};

}
