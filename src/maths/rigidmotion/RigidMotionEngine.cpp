#include <helios/maths/rigidmotion/RigidMotionEngine.h>
#include <helios/maths/rigidmotion/RigidMotionException.h>

using namespace rigidmotion;

// ***  RIGID MOTION ENGINE METHODS  *** //
// ************************************* //
arma::mat
RigidMotionEngine::apply(RigidMotion const& f, arma::mat const& X)
{
  arma::mat Y = f.getA() * X;
  return f.getC() + Y.each_col();
}

arma::colvec
RigidMotionEngine::apply(RigidMotion const& f, arma::colvec const& X)
{
  return f.getC() + f.getA() * X;
}

RigidMotion
RigidMotionEngine::compose(RigidMotion const& f, RigidMotion const& g)
{
  return f.compose(g);
}

arma::mat
RigidMotionEngine::computeFixedPoints(RigidMotion const& f,
                                      std::size_t& dim,
                                      bool safe)
{
  // When safe mode is requested, check that indeed there are fixed points
  if (safe) {
    if (!f.hasFixedPoints()) {
      throw RigidMotionException(
        "RigidMotionEngine cannot compute fixed points for a rigid "
        "motion with no fixed points");
    }
  }

  // Find space dimensionality and invariant dimensionality
  std::size_t n = f.getDimensionality();
  dim = f.findInvariantDimensionality();
  std::size_t eigenvecIndex = 0;
  if (dim == 1)
    eigenvecIndex = n - 1;

  // The set of fixed points is the exact solution for the system (1 point)
  if (dim == 0) {
    return solve(arma::eye(n, n) - f.getA(), f.getC());
  }

  // The set of fixed points is a lineal variety with dimensionality > 0
  arma::colvec s;
  arma::mat U, V;
  svd_econ(U, s, V, arma::eye(n, n) - f.getA(), "left", "std");
  if (dim < n)
    return U.col(eigenvecIndex);
  return U;
}

arma::mat
RigidMotionEngine::computeAssociatedInvariant(RigidMotion const& f,
                                              std::size_t& dim,
                                              bool safe)
{
  // When safe mode, check that indeed there is an associated invariant
  if (safe) {
    if (f.hasFixedPoints()) {
      throw RigidMotionException(
        "RigidMotionEngine cannot compute associated invariant for a "
        "rigid motion with fixed points");
    }
  }

  // Find space dimensionality and invariant dimensionality
  std::size_t n = f.getDimensionality();
  dim = f.findInvariantDimensionality();
  arma::mat I = arma::eye(n, n);
  arma::mat A = f.getA();
  std::size_t eigenvecIndex = 0;
  if (dim == 1)
    eigenvecIndex = n - 1;

  // The associated invariant is the exact solution for the system (1 point)
  if (dim == 0) { // There is no rigid motion in R2 nor R3 with point as invar.
    return solve((A - I) * (A - I), (I - A) * f.getC());
  }

  // The associated invariant is a lineal variety with dimensionality > 0
  arma::colvec s;
  arma::mat U, V;
  svd_econ(U, s, V, I - A);
  if (dim < n)
    return U.col(eigenvecIndex);
  return U;
}
