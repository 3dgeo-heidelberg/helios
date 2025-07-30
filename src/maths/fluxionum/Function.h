#pragma once

namespace fluxionum {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class representing a function
 *
 * A function \f$f\f$ is a rule of assignment \f$r\f$, together with a set
 *  \f$B\f$ that contains the image set of \f$r\f$. The domain \f$A\f$ of the
 *  rule \f$r\f$ is also called the <b>domain</b> of the function \f$f\f$;
 *  the image set of \f$r\f$ is also called the <b>image set</b> of \f$f\f$;
 *  and the set \f$B\f$ is called the <b>range</b> of \f$f\f$.
 *
 * A <b>rule of assignment</b> is a subset \f$r\f$ of the cartesian product
 *  \f$C \times D\f$ of two sets, having the property that each element of C
 *  appears as the first coordinate of at most one ordered pair belonging to
 *  \f$r\f$.
 *
 * @tparam A The domain of the function
 * @tparam B The range of the function
 * @see Topology, James Munkres, second edition PEARSON (pages 13 and 14)
 */
template<typename A, typename B>
class Function
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @bried Function default constructor
   */
  Function() = default;
  virtual ~Function() = default;

  // ***  FUNCTION METHODS  *** //
  // ************************** //
  /**
   * @brief Evaluate the function \f$f(x)=y\f$
   * @param x The input from the domain: \f$x \in A\f$
   * @return The output from the range: \f$y \in B\f$
   */
  virtual B eval(A const& x) = 0;

  // ***  FUNCTION OPERATORS  *** //
  // **************************** //
  /**
   * @brief Evaluate the function \f$f(x)=y\f$
   * @param x The input from the domain: \f$x \in A\f$
   * @return The output from the range: \f$y \in B\f$
   * @see fluxionum::Function::eval
   */
  inline B operator()(A const& x) { return eval(x); }
};

}
