#include <rigidmotion/RigidMotionR2Factory.h>

using namespace rigidmotion;

// ***  RIGID MOTION FACTORY METHODS  *** //
// ************************************** //
RigidMotion
RigidMotionR2Factory::makeIdentity() const
{
  return RigidMotion(arma::zeros(2), arma::eye(2, 2));
};

RigidMotion
RigidMotionR2Factory::makeTranslation(arma::colvec const shift) const
{
  return RigidMotion(shift, arma::eye(2, 2));
}

RigidMotion
RigidMotionR2Factory::makeReflection(arma::colvec const axis) const
{
  return makeReflection(std::atan2(axis[1], axis[0]));
}

RigidMotion
RigidMotionR2Factory::makeReflection(double const theta) const
{
  double const theta2 = 2.0 * theta;
  double const theta2cos = std::cos(theta2);
  double const theta2sin = std::sin(theta2);
  arma::mat A = arma::mat(2, 2);
  A.at(0, 0) = theta2cos;
  A.at(0, 1) = theta2sin;
  A.at(1, 0) = theta2sin;
  A.at(1, 1) = -theta2cos;
  arma::colvec C = arma::colvec(2, arma::fill::zeros);
  return RigidMotion(C, A);
}

RigidMotion
RigidMotionR2Factory::makeGlideReflection(arma::colvec const axis,
                                          double const glide) const
{
  return makeGlideReflection(std::atan2(axis[1], axis[0]), glide);
}

RigidMotion
RigidMotionR2Factory::makeGlideReflection(double const theta,
                                          double const glide) const
{
  arma::colvec const axis(
    std::vector<double>({ std::cos(theta), std::sin(theta) }));
  RigidMotion rm = makeReflection(theta);
  rm.setC(glide * axis);
  return rm;
}

RigidMotion
RigidMotionR2Factory::makeRotation(double const theta,
                                   arma::colvec const center) const
{
  double const thetacos = std::cos(theta);
  double const thetasin = std::sin(theta);
  arma::mat A = arma::mat(2, 2);
  A.at(0, 0) = thetacos;
  A.at(0, 1) = -thetasin;
  A.at(1, 0) = thetasin;
  A.at(1, 1) = thetacos;
  return RigidMotion((arma::eye(2, 2) - A) * center, A);
}
