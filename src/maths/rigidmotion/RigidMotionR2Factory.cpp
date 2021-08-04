#include <rigidmotion/RigidMotionR2Factory.h>

using namespace rigidmotion;

// ***  RIGID MOTION FACTORY METHODS  *** //
// ************************************** //
RigidMotion RigidMotionR2Factory::makeIdentity(){
    mat A = mat(2, 2, arma::fill::eye);
    colvec C = colvec(2, arma::fill::zeros);
    return RigidMotion(C, A);
};

RigidMotion RigidMotionR2Factory::makeTranslation(arma::colvec const shift){
    mat A = arma::mat(2, 2, arma::fill::eye);
    colvec C = shift;
    return RigidMotion(C, A);
}

RigidMotion RigidMotionR2Factory::makeReflection(colvec const axis){
    return makeReflection(std::atan2(axis[1], axis[0]));
}

RigidMotion RigidMotionR2Factory::makeReflection(double const theta){
    double const theta2 = 2.0*theta;
    double const theta2cos = std::cos(theta2);
    double const theta2sin = std::sin(theta2);
    mat A = mat(2, 2);
    A.at(0, 0) = theta2cos;
    A.at(0, 1) = theta2sin;
    A.at(1, 0) = theta2sin;
    A.at(1, 1) = -theta2cos;
    colvec C = colvec(2, arma::fill::zeros);
    return RigidMotion(C, A);
}

RigidMotion RigidMotionR2Factory::makeGlideReflection(
    colvec const axis,
    double const glide
){
    return makeGlideReflection(std::atan2(axis[1], axis[0]), glide);
}

RigidMotion RigidMotionR2Factory::makeGlideReflection(
    double const theta,
    double const glide
){
    colvec const axis(std::vector<double>({std::cos(theta), std::sin(theta)}));
    RigidMotion rm = makeReflection(theta);
    rm.setC(glide * axis);
    return rm;
}

RigidMotion RigidMotionR2Factory::makeRotation(
    double const theta,
    colvec const center
){
    mat A = mat(2, 2);
    A.at(0, 0) = std::cos(theta);
    A.at(0, 1) = -std::sin(theta);
    A.at(1, 0) = std::sin(theta);
    A.at(1, 1) = std::cos(theta);
    return RigidMotion((eye(2, 2)-A)*center, A);
}