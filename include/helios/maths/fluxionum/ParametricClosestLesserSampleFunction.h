#pragma once

#include <helios/maths/fluxionum/Function.h>

#include <armadillo>

namespace fluxionum {

template<typename A, typename B>
class ParametricClosestLesserSampleFunction : public Function<A, arma::Col<B>>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The vector of sorted domain components
   *  \f$\vec{t} = (t_1, \ldots, t_m)\f$.
   */
  arma::Col<A> const& t;
  /**
   * @brief The matrix of sorted image components
   *  \f$\mathbf{Y} \in \mathbb{R}^{m \times n}\f$ such that
   *  \f$\forall i,\, \vec{y}(t_{i}) = \vec{y_i}\f$, where \f$\vec{y_i}\f$ is
   *  the \f$i\f$-th row of \f$\mathbf{Y}\f$ as row vector
   */
  arma::Mat<B> const& y;
  /**
   * @brief The index of the current sorted domain sample
   */
  size_t i;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief ParametricClosestLesserSampleFunction default constructor
   * @param i The index of the initial domain sample. By default, it is
   *  assumed to be 0, it is the first domain sample.
   * @see fluxionum::ParametricClosestLesserSampleFunction::t
   * @see fluxionum::ParametricClosestLesserSampleFunction::y
   * @see fluxionum::ParametricClosestLesserSampleFunction::i
   */
  ParametricClosestLesserSampleFunction(arma::Col<A> const& t,
                                        arma::Mat<B> const& y,
                                        size_t const i = 0)
    : t(t)
    , y(y)
    , i(i)
  {
  }
  virtual ~ParametricClosestLesserSampleFunction() = default;

  // ***  ASSIGNMENT  *** //
  // ******************** //
  /**
   * @brief Assignment reference operator
   */
  ParametricClosestLesserSampleFunction<A, B>& operator=(
    ParametricClosestLesserSampleFunction<A, B> const& pclsf)
  {
    this->~ParametricClosestLesserSampleFunction<A, B>();
    new (this)
      ParametricClosestLesserSampleFunction<A, B>(pclsf.t, pclsf.y, pclsf.i);
    return *this;
  }

  // ***  FUNCTION METHODS  *** //
  // ************************** //
  /**
   * @brief Find the sample with closest lesser domain with respect to given
   *  \f$t\f$ and return its known vector image \f$\vec{y_i}\f$
   * @param tx The domain value which closest lesser sample must be found
   * @return \f$\vec{y_i}\f$, it is the known vector image of the closest
   *  lesser sample
   */
  arma::Col<B> eval(A const& tx) override
  {
    // Update index if necessary
    if ((i < (t.n_elem) - 1) && (tx >= t.at(i + 1) || tx < t.at(i))) {
      if (i < (t.n_elem - 2) && tx < t.at(i + 2))
        ++i;
      else {
        i = 0;
        for (size_t j = t.n_elem - 1; j > 0; --j) {
          if (tx >= t.at(j)) {
            i = j;
            break;
          }
        }
      }
    }
    // Return the image of the closest lesser sample
    return y.row(i).as_col();
  }

  /**
   * @brief Restart the ParametricClosestLesserSampleFunction so it is at its
   *  initial state again (it is, at the first sample)
   */
  virtual void restart() { setCurrentSampleIndex(0); }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @see fluxionum::ParametricClosestLesserSampleFunction::t
   */
  inline arma::Col<A> const& getT() const { return t; }
  /**
   * @see fluxionum::ParametricClosestLesserSampleFunction::y
   */
  inline arma::Mat<B> const& getY() const { return y; }
  /**
   * @see fluxionum::ParametricClosestLesserSampleFunction::i
   */
  inline size_t getCurrentSampleIndex() const { return i; }
  /**
   * @see fluxionum::ParametricClosestLesserSampleFunction::i
   */
  inline void setCurrentSampleIndex(size_t const i) { this->i = i; }
};

}
