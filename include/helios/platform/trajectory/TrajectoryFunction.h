#pragma once

#include <helios/maths/fluxionum/Function.h>

#include <armadillo>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class representing a trajectory function
 * \f$f: \mathbb{R} \to \mathbb{R}^{n}\f$.
 *
 * A trajectory function \f$f(t) \in \mathbb{R}^{n}\f$ is a parametric function
 *  that for each time \f$t\f$ defines a trajectory of \f$n\f$ components.
 *  It is, \f$f(t) = (x_1, \ldots, x_n)\f$.
 *
 * Any concrete implementation of a TrajectoryFunction must provide a valid
 *  implementation of the fluxionum::Function::eval method
 *
 * @see fluxionum::Function
 * @see fluxionum::Function::eval
 * @see fluxionum::Function::operator()
 */
class TrajectoryFunction : public fluxionum::Function<double, arma::Col<double>>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for TrajectoryFunction
   */
  TrajectoryFunction() = default;
  virtual ~TrajectoryFunction() = default;
};
