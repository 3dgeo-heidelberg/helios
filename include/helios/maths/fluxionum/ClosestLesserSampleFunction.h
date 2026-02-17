#pragma once

#include <helios/maths/fluxionum/Function.h>

#include <armadillo>

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Closest lesser sample function
 *
 * The ClosestLesserSampleFunction must have a vector of sorted domain
 *  components such that \f$\forall i,\, t_{i+1} > t_{i}\f$. Also, for each
 *  \f$t_i\f$ there must exist a known
 *  \f$y_i\f$ such that \f$y(t_{i}) = y_i\f$.
 *
 * For the sake of efficiency, the ClosestLesserSampleFunction assumes that
 *  the value of \f$t\f$ is going to be queried in an unidirectional forward
 *  sense. When querying values in a backward sense, result integrity is
 *  guaranteed but it might be at the expenses of performance.
 *
 * @see fluxionum::Function
 */
template<typename A, typename B>
class ClosestLesserSampleFunction : public Function<A, B>
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
   * @brief The vector of sorted image components
   *  \f$\vec{y} = (y_1, \ldots, y_m)\f$ such that
   *  \f$\forall i,\, y(t_{i}) = y_i\f$
   */
  arma::Col<B> const& y;
  /**
   * @brief The index of the current sorted domain sample
   */
  std::size_t i;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief ClosestLesserSampleFunction default constructor
   * @param i The index of the initial domain sample. By default, it is
   *  assumed to be 0, it is the first domain sample.
   * @see fluxionum::ClosestLesserSampleFunction::t
   * @see fluxionum::ClosestLesserSampleFunction::y
   * @see fluxionum::ClosestLesserSampleFunction::i
   */
  ClosestLesserSampleFunction(arma::Col<A> const& t,
                              arma::Col<B> const& y,
                              std::size_t const i = 0)
    : t(t)
    , y(y)
    , i(i)
  {
  }
  virtual ~ClosestLesserSampleFunction() = default;

  // ***  ASSIGNMENT  *** //
  // ******************** //
  /**
   * @brief Assignment reference operator
   */
  ClosestLesserSampleFunction<A, B>& operator=(
    ClosestLesserSampleFunction<A, B> const& clsf)
  {
    this->~ClosestLesserSampleFunction<A, B>();
    new (this) ClosestLesserSampleFunction<A, B>(clsf.t, clsf.y, clsf.i);
    return *this;
  }

  // ***  FUNCTION METHODS  *** //
  // ************************** //
  /**
   * @brief Find the sample with closest lesser domain with respect to given
   *  \f$t\f$ and return its known image \f$y_i\f$
   * @param tx The domain value which closest lesser sample must be found
   * @return \f$y_i\f$, it is the known image of the closest lesser sample
   */
  B eval(A const& tx) override
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
    return y.at(i);
  }

  /**
   * @brief Restart the ClosestLesserSampleFunction so it is at its
   *  initial state again (it is, at the first sample)
   */
  virtual void restart() { setCurrentSampleIndex(0); }

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @see fluxionum::ClosestLesserSampleFunction::t
   */
  inline arma::Col<A> const& getT() const { return t; }
  /**
   * @see fluxionum::ClosestLesserSampleFunction::y
   */
  inline arma::Col<B> const& getY() const { return y; }
  /**
   * @see fluxionum::ClosestLesserSampleFunction::i
   */
  inline std::size_t getCurrentSampleIndex() const { return i; }
  /**
   * @see fluxionum::ClosestLesserSampleFunction::i
   */
  inline void setCurrentSampleIndex(std::size_t const i) { this->i = i; }
};

}
