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
