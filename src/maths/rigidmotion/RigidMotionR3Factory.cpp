#include <maths/MathConstants.h>
#include <rigidmotion/RigidMotionException.h>
#include <rigidmotion/RigidMotionR3Factory.h>

using namespace rigidmotion;

// ***  CONSTANTS  *** //
// ******************* //
double const RigidMotionR3Factory::eps = 0.000000001;
arma::mat const RigidMotionR3Factory::canonicalReflection =
  arma::mat("-1 0 0; 0 1 0; 0 0 1");

// ***  RIGID MOTION FACTORY METHODS  *** //
// ************************************** //
RigidMotion
RigidMotionR3Factory::makeIdentity() const
{
  return RigidMotion(arma::zeros(3), arma::eye(3, 3));
}

RigidMotion
RigidMotionR3Factory::makeTranslation(arma::colvec const shift) const
{
  return RigidMotion(shift, arma::eye(3, 3));
}

RigidMotion
RigidMotionR3Factory::makeReflection(arma::colvec const ortho) const
{
  return makeReflectionFast(arma::normalise(ortho));
}

RigidMotion
RigidMotionR3Factory::makeReflectionFast(arma::colvec const orthonormal) const
{
  // Compute alpha vector
  arma::colvec alpha(3);
  if (orthonormal[0] == 0.0 && orthonormal[1] == 0.0) {
    alpha[0] = SQRT2;
    alpha[1] = SQRT2;
    alpha[2] = 0;
  } else if (orthonormal[1] == 0.0) {
    alpha[0] = -orthonormal(2) / orthonormal(0);
    alpha[1] = 1;
    alpha[2] = 1;
    alpha = arma::normalise(alpha);
  } else {
    alpha[0] = 1;
    alpha[1] = (-orthonormal[0] - orthonormal[2]) / orthonormal[1];
    alpha[2] = 1;
    alpha = arma::normalise(alpha);
  }

  // Compute beta vector
  arma::colvec beta = arma::normalise(arma::cross(orthonormal, alpha));

  // Build reflection
  return makeReflectionFast(orthonormal, alpha, beta);
}

RigidMotion
RigidMotionR3Factory::makeReflectionFast(arma::colvec const u,
                                         arma::colvec const alpha,
                                         arma::colvec const beta) const
{
  // Build basis matrix
  arma::mat B(3, 3);
  B.col(0) = u;
  B.col(1) = alpha;
  B.col(2) = beta;

  // Inverse of basis matrix
  arma::mat Binv = inv(B);

  // Build reflection
  return RigidMotion(arma::zeros(3), B * canonicalReflection * Binv);
}

RigidMotion
RigidMotionR3Factory::makeReflectionX() const
{
  return RigidMotion(arma::zeros(3), canonicalReflection);
}

RigidMotion
RigidMotionR3Factory::makeReflectionY() const
{
  return RigidMotion(arma::zeros(3), arma::mat("1 0 0; 0 -1 0; 0 0 1"));
}

RigidMotion
RigidMotionR3Factory::makeReflectionZ() const
{
  return RigidMotion(arma::zeros(3), arma::mat("1 0 0; 0 1 0; 0 0 -1"));
}

RigidMotion
RigidMotionR3Factory::makeGlideReflection(arma::colvec const ortho,
                                          arma::colvec const shift) const
{
  return makeGlideReflectionFast(arma::normalise(ortho), shift);
}

RigidMotion
RigidMotionR3Factory::makeGlideReflectionFast(arma::colvec const orthonormal,
                                              arma::colvec const shift) const
{
  // Check shift is orthogonal to plane orthonormal
  if (std::fabs(arma::dot(orthonormal, shift)) > eps) {
    throw RigidMotionException(
      "RigidMotionR3Factory failed to build glide reflection.\n"
      "Shift vector was not contained in the reflection plane");
  }

  // Build the rigid motion
  RigidMotion rm = makeReflectionFast(orthonormal);
  rm.setC(shift);
  return rm;
}

RigidMotion
RigidMotionR3Factory::makeRotation(arma::colvec const axis,
                                   double const theta) const
{
  return makeRotationFast(arma::normalise(axis), theta);
}

RigidMotion
RigidMotionR3Factory::makeRotationFast(arma::colvec const axis,
                                       double const theta) const
{
  // Cache partial computations
  double const uxuy = axis(0) * axis(1);
  double const uxuz = axis(0) * axis(2);
  double const uyuz = axis(1) * axis(2);
  double const tcos = std::cos(theta);
  double const ccos = 1 - tcos;
  double const tsin = std::sin(theta);
  double const uxuyccos = uxuy * ccos;
  double const uxuzccos = uxuz * ccos;
  double const uyuzccos = uyuz * ccos;
  double const uxtsin = axis(0) * tsin;
  double const uytsin = axis(1) * tsin;
  double const uztsin = axis(2) * tsin;

  // Compute the rotation matrix
  arma::mat A(3, 3);
  A.at(0, 0) = tcos + axis(0) * axis(0) * ccos;
  A.at(0, 1) = uxuyccos - uztsin;
  A.at(0, 2) = uxuzccos + uytsin;
  A.at(1, 0) = uxuyccos + uztsin;
  A.at(1, 1) = tcos + axis(1) * axis(1) * ccos;
  A.at(1, 2) = uyuzccos - uxtsin;
  A.at(2, 0) = uxuzccos - uytsin;
  A.at(2, 1) = uyuzccos + uxtsin;
  A.at(2, 2) = tcos + axis(2) * axis(2) * ccos;

  // Build the rigid motion
  return RigidMotion(arma::zeros(3), A);
}

RigidMotion
RigidMotionR3Factory::makeRotationX(double const theta) const
{
  // Cache partial computations
  double const thetacos = std::cos(theta);
  double const thetasin = std::sin(theta);

  // Compute the rotation matrix
  arma::mat A = arma::eye(3, 3);
  A.at(1, 1) = thetacos;
  A.at(1, 2) = -thetasin;
  A.at(2, 1) = thetasin;
  A.at(2, 2) = thetacos;

  // Build the rigid motion
  return RigidMotion(arma::zeros(3), A);
}

RigidMotion
RigidMotionR3Factory::makeRotationY(double const theta) const
{
  // Cache partial computations
  double const thetacos = std::cos(theta);
  double const thetasin = std::sin(theta);

  // Compute the rotation matrix
  arma::mat A = arma::eye(3, 3);
  A.at(0, 0) = thetacos;
  A.at(0, 2) = thetasin;
  A.at(2, 0) = -thetasin;
  A.at(2, 2) = thetacos;

  // Build the rigid motion
  return RigidMotion(arma::zeros(3), A);
}

RigidMotion
RigidMotionR3Factory::makeRotationZ(double const theta) const
{
  // Cache partial computations
  double const thetacos = std::cos(theta);
  double const thetasin = std::sin(theta);

  // Compute the rotation matrix
  arma::mat A = arma::eye(3, 3);
  A.at(0, 0) = thetacos;
  A.at(0, 1) = -thetasin;
  A.at(1, 0) = thetasin;
  A.at(1, 1) = thetacos;

  // Build the rigid motion
  return RigidMotion(arma::zeros(3), A);
}

RigidMotion
RigidMotionR3Factory::makeHelical(arma::colvec const axis,
                                  double const theta,
                                  double const glide) const
{
  return makeHelicalFast(arma::normalise(axis), theta, glide);
}

RigidMotion
RigidMotionR3Factory::makeHelicalFast(arma::colvec const axis,
                                      double const theta,
                                      double const glide) const
{
  RigidMotion rm = makeRotationFast(axis, theta);
  rm.setC(glide * axis);
  return rm;
}

RigidMotion
RigidMotionR3Factory::makeHelicalX(double const theta, double const glide) const
{
  RigidMotion rm = makeRotationX(theta);
  rm.setC(arma::colvec(std::vector<double>({ glide, 0, 0 })));
  return rm;
}

RigidMotion
RigidMotionR3Factory::makeHelicalY(double const theta, double const glide) const
{
  RigidMotion rm = makeRotationY(theta);
  rm.setC(arma::colvec(std::vector<double>({ 0, glide, 0 })));
  return rm;
}

RigidMotion
RigidMotionR3Factory::makeHelicalZ(double const theta, double const glide) const
{
  RigidMotion rm = makeRotationZ(theta);
  rm.setC(arma::colvec(std::vector<double>({ 0, 0, glide })));
  return rm;
}

RigidMotion
RigidMotionR3Factory::makeRotationalSymmetry(arma::colvec const axis,
                                             double const theta) const
{
  return makeRotationalSymmetryFast(arma::normalise(axis), theta);
}

RigidMotion
RigidMotionR3Factory::makeRotationalSymmetryFast(arma::colvec const axis,
                                                 double const theta) const
{
  return makeReflectionFast(axis).compose(makeRotation(axis, theta));
}

RigidMotion
RigidMotionR3Factory::makeRotationalSymmetryX(double const theta) const
{
  return makeReflectionX().compose(makeRotationX(theta));
}

RigidMotion
RigidMotionR3Factory::makeRotationalSymmetryY(double const theta) const
{
  return makeReflectionY().compose(makeRotationY(theta));
}

RigidMotion
RigidMotionR3Factory::makeRotationalSymmetryZ(double const theta) const
{
  return makeReflectionZ().compose(makeRotationZ(theta));
}

RigidMotion
RigidMotionR3Factory::makeRotationalSymmetry(arma::colvec const axis,
                                             double const theta,
                                             arma::colvec const center) const
{
  return makeRotationalSymmetryFast(arma::normalise(axis), theta, center);
}

RigidMotion
RigidMotionR3Factory::makeRotationalSymmetryFast(
  arma::colvec const axis,
  double const theta,
  arma::colvec const center) const
{
  RigidMotion rm = makeRotationalSymmetryFast(axis, theta);
  rm.setC((arma::eye(3, 3) - rm.getA()) * center);
  return rm;
}
