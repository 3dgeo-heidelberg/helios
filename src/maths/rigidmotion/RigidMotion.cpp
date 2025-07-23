#include <rigidmotion/RigidMotion.h>
#include <rigidmotion/RigidMotionException.h>
#include <sstream>

using namespace rigidmotion;

// ***  CONSTANTS  *** //
// ******************* //
double const RigidMotion::eps = 0.000000001;

// ***  RIGID MOTION METHODS  *** //
// ****************************** //
RigidMotion
RigidMotion::compose(RigidMotion const rm) const
{
  RigidMotion comp;
  comp.C = C + A * rm.C;
  comp.A = A * rm.A;
  return comp;
}

void
RigidMotion::composeInPlace(RigidMotion const rm)
{
  *this = compose(rm);
}

RigidMotion::SuperType
RigidMotion::findSuperType() const
{
  size_t const n = getDimensionality();
  size_t const r = arma::rank(arma::eye(n, n) - A, eps);
  bool rankMismatchesDimensionality = false;
  if (n == 2) {
    if (r == 0)
      return SuperType::R2_BASE;
    else if (r == 1)
      return SuperType::R2_REFLECTION;
    else if (r == 2)
      return SuperType::R2_ROTATION;
    else
      rankMismatchesDimensionality = true;
  } else if (n == 3) {
    if (r == 0)
      return SuperType::R3_BASE;
    else if (r == 1)
      return SuperType::R3_REFLECTION;
    else if (r == 2)
      return SuperType::R3_ROTATION;
    else if (r == 3)
      return SuperType::R3_ROTATIONAL_SYMMETRY;
    else
      rankMismatchesDimensionality = true;
  } else {
    std::stringstream ss;
    ss << "Rigid motion super type could not be determined because "
       << "unexpected dimensionality: " << n;
    throw RigidMotionException(ss.str());
  }

  if (rankMismatchesDimensionality) {
    std::stringstream ss;
    ss << "Rigid motion super type could not be determined because "
       << "unexpected rank " << r << " for dimensionality " << n;
    throw RigidMotionException(ss.str());
  }

  // Below code is never reached but it is necessary to prevent certain
  // compilers to raise non-void function with no return warnings
  throw RigidMotionException(
    "Unexpected behavior for rigidmotion::RigidMotion::findSuperType "
    "method");
}

bool
RigidMotion::hasFixedPoints() const
{
  std::size_t n = getDimensionality();
  arma::mat coef = (arma::eye(n, n) - A);   // coef = I-A
  arma::mat sys = arma::join_rows(coef, C); // sys = (I-A | C)
  return arma::rank(coef, eps) == arma::rank(sys, eps);
}

RigidMotion::Type
RigidMotion::findType() const
{
  RigidMotion::SuperType st = findSuperType();
  bool fixedPoints = hasFixedPoints();
  switch (st) {
    default:
      throw RigidMotionException(
        "Unexpected type configuration for rigid motion");
    case R2_BASE:
      if (fixedPoints)
        return IDENTITY_R2;
      return TRANSLATION_R2;
    case R2_REFLECTION:
      if (fixedPoints)
        return REFLECTION_R2;
      return GLIDE_REFLECTION_R2;
    case R2_ROTATION:
      if (fixedPoints)
        return ROTATION_R2;
      throw RigidMotionException(
        "Unexpected type configuration for R2 rotation rigid motion");
    case R3_BASE:
      if (fixedPoints)
        return IDENTITY_R3;
      return TRANSLATION_R3;
    case R3_REFLECTION:
      if (fixedPoints)
        return REFLECTION_R3;
      return GLIDE_REFLECTION_R3;
    case R3_ROTATION:
      if (fixedPoints)
        return ROTATION_R3;
      return HELICAL_R3;
    case R3_ROTATIONAL_SYMMETRY:
      if (fixedPoints)
        return ROTATIONAL_SYMMETRY_R3;
      throw RigidMotionException(
        "Unexpected type configuration for R3 rotational symmetry "
        "rigid motion");
  }
}

std::size_t
RigidMotion::findInvariantDimensionality() const
{
  std::size_t n = getDimensionality();
  return n - arma::rank(arma::eye(n, n) - A, eps);
}
