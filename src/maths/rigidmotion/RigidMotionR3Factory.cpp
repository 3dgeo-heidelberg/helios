#include <rigidmotion/RigidMotionR3Factory.h>
#include <rigidmotion/RigidMotionException.h>
#include <maths/MathConstants.h>

using namespace rigidmotion;

// ***  CONSTANTS  *** //
// ******************* //
double const RigidMotionR3Factory::eps = 0.000000001;
mat const RigidMotionR3Factory::canonicalReflection = mat(
    "-1 0 0; 0 1 0; 0 0 1"
);


// ***  RIGID MOTION FACTORY METHODS  *** //
// ************************************** //
RigidMotion RigidMotionR3Factory::makeIdentity(){
    return RigidMotion(zeros(3), eye(3, 3));
}

RigidMotion RigidMotionR3Factory::makeTranslation(colvec const shift){
    return RigidMotion(shift, eye(3, 3));
}

RigidMotion RigidMotionR3Factory::makeReflection(colvec const ortho){
    return makeReflectionFast(normalise(ortho));
}

RigidMotion RigidMotionR3Factory::makeReflectionFast(colvec const orthonormal){
    // Compute alpha vector
    colvec alpha(3);
    if(orthonormal[0] == 0.0 && orthonormal[1] == 0.0){
        alpha[0] = SQRT2;
        alpha[1] = SQRT2;
        alpha[2] = 0;
    }
    else if(orthonormal[1] == 0.0){
        alpha[0] = -orthonormal(2)/orthonormal(0);
        alpha[1] = 1;
        alpha[2] = 1;
        alpha = normalise(alpha);
    }
    else{
        alpha[0] = 1;
        alpha[1] = (-orthonormal[0]-orthonormal[2])/orthonormal[1];
        alpha[2] = 1;
        alpha = normalise(alpha);
    }

    // Compute beta vector
    colvec beta = normalise(cross(orthonormal, alpha));

    // Build reflection
    return makeReflectionFast(orthonormal, alpha, beta);
}

RigidMotion RigidMotionR3Factory::makeReflectionFast(
    colvec const u,
    colvec const alpha,
    colvec const beta
){
    // Build basis matrix
    mat B(3, 3);
    B.col(0) = u;
    B.col(1) = alpha;
    B.col(2) = beta;

    // Inverse of basis matrix
    mat Binv = inv(B);

    // Build reflection
    return RigidMotion(zeros(3), B*canonicalReflection*Binv);

}

RigidMotion RigidMotionR3Factory::makeReflectionX(){
    return RigidMotion(zeros(3), canonicalReflection);
}

RigidMotion RigidMotionR3Factory::makeReflectionY(){
    return RigidMotion(zeros(3), mat("1 0 0; 0 -1 0; 0 0 1"));
}

RigidMotion RigidMotionR3Factory::makeReflectionZ(){
    return RigidMotion(zeros(3), mat("1 0 0; 0 1 0; 0 0 -1"));
}

RigidMotion RigidMotionR3Factory::makeGlideReflection(
    colvec const ortho,
    colvec const shift
){
    return makeGlideReflectionFast(normalise(ortho), shift);
}

RigidMotion RigidMotionR3Factory::makeGlideReflectionFast(
    colvec const orthonormal,
    colvec const shift
){
    // Check shift is orthogonal to plane orthonormal
    if(std::fabs(dot(orthonormal, shift)) > eps){
        throw RigidMotionException(
            "RigidMotionR3Factory failed to build glide reflection.\n"
            "Shift vector was not contained in the reflection plane"
        );
    }

    // Build the rigid motion
    RigidMotion rm = makeReflectionFast(orthonormal);
    rm.setC(shift);
    return rm;
}

RigidMotion RigidMotionR3Factory::makeRotation(
    colvec const axis,
    double const theta
){
    return makeRotationFast(normalise(axis), theta);
}

RigidMotion RigidMotionR3Factory::makeRotationFast(
    colvec const axis,
    double const theta
){
    // Cache partial computations
    double const uxuy = axis(0)*axis(1);
    double const uxuz = axis(0)*axis(2);
    double const uyuz = axis(1)*axis(2);
    double const tcos = std::cos(theta);
    double const ccos = 1-tcos;
    double const tsin = std::sin(theta);
    double const uxuyccos = uxuy*ccos;
    double const uxuzccos = uxuz*ccos;
    double const uyuzccos = uyuz*ccos;
    double const uxtsin = axis(0)*tsin;
    double const uytsin = axis(1)*tsin;
    double const uztsin = axis(2)*tsin;

    // Compute the rotation matrix
    mat A(3, 3);
    A.at(0, 0) = tcos + axis(0)*axis(0)*ccos;
    A.at(0, 1) = uxuyccos - uztsin;
    A.at(0, 2) = uxuzccos + uytsin;
    A.at(1, 0) = uxuyccos + uztsin;
    A.at(1, 1) = tcos + axis(1)*axis(1)*ccos;
    A.at(1, 2) = uyuzccos - uxtsin;
    A.at(2, 0) = uxuzccos - uytsin;
    A.at(2, 1) = uyuzccos + uxtsin;
    A.at(2, 2) = tcos + axis(2)*axis(2)*ccos;

    // Build the rigid motion
    return RigidMotion(zeros(3), A);
}

RigidMotion RigidMotionR3Factory::makeRotationX(double const theta){
    // Cache partial computations
    double const thetacos = std::cos(theta);
    double const thetasin = std::sin(theta);

    // Compute the rotation matrix
    mat A = eye(3, 3);
    A.at(1, 1) = thetacos;
    A.at(1, 2) = -thetasin;
    A.at(2, 1) = thetasin;
    A.at(2, 2) = thetacos;

    // Build the rigid motion
    return RigidMotion(zeros(3), A);
}

RigidMotion RigidMotionR3Factory::makeRotationY(double const theta){
    // Cache partial computations
    double const thetacos = std::cos(theta);
    double const thetasin = std::sin(theta);

    // Compute the rotation matrix
    mat A = eye(3, 3);
    A.at(0, 0) = thetacos;
    A.at(0, 2) = thetasin;
    A.at(2, 0) = -thetasin;
    A.at(2, 2) = thetacos;

    // Build the rigid motion
    return RigidMotion(zeros(3), A);
}

RigidMotion RigidMotionR3Factory::makeRotationZ(double const theta){
    // Cache partial computations
    double const thetacos = std::cos(theta);
    double const thetasin = std::sin(theta);

    // Compute the rotation matrix
    mat A = eye(3, 3);
    A.at(0, 0) = thetacos;
    A.at(0, 1) = -thetasin;
    A.at(1, 0) = thetasin;
    A.at(1, 1) = thetacos;

    // Build the rigid motion
    return RigidMotion(zeros(3), A);
}

RigidMotion RigidMotionR3Factory::makeHelical(
    colvec const axis,
    double const theta,
    double const glide
){
    return makeHelicalFast(normalise(axis), theta, glide);
}

RigidMotion RigidMotionR3Factory::makeHelicalFast(
    colvec const axis,
    double const theta,
    double const glide
){
    RigidMotion rm = makeRotationFast(axis, theta);
    rm.setC(glide*axis);
    return rm;
}

RigidMotion RigidMotionR3Factory::makeHelicalX(
    double const theta,
    double const glide
){
    RigidMotion rm = makeRotationX(theta);
    rm.setC(colvec(std::vector<double>({glide, 0, 0})));
    return rm;
}

RigidMotion RigidMotionR3Factory::makeHelicalY(
    double const theta,
    double const glide
){
    RigidMotion rm = makeRotationY(theta);
    rm.setC(colvec(std::vector<double>({0, glide, 0})));
    return rm;
}

RigidMotion RigidMotionR3Factory::makeHelicalZ(
    double const theta,
    double const glide
){
    RigidMotion rm = makeRotationZ(theta);
    rm.setC(colvec(std::vector<double>({0, 0, glide})));
    return rm;
}

RigidMotion RigidMotionR3Factory::makeRotationalSymmetry(
    colvec const axis,
    double const theta
){
    return makeRotationalSymmetryFast(normalise(axis), theta);
}

RigidMotion RigidMotionR3Factory::makeRotationalSymmetryFast(
    colvec const axis,
    double const theta
){
    return makeReflectionFast(axis).compose(makeRotation(axis, theta));
}

RigidMotion RigidMotionR3Factory::makeRotationalSymmetryX(double const theta){
    return makeReflectionX().compose(makeRotationX(theta));
}

RigidMotion RigidMotionR3Factory::makeRotationalSymmetryY(double const theta){
    return makeReflectionY().compose(makeRotationY(theta));

}

RigidMotion RigidMotionR3Factory::makeRotationalSymmetryZ(double const theta){
    return makeReflectionZ().compose(makeRotationZ(theta));
}

RigidMotion RigidMotionR3Factory::makeRotationalSymmetry(
    colvec const axis,
    double const theta,
    colvec const center
){
    return makeRotationalSymmetryFast(normalise(axis), theta, center);
}

RigidMotion RigidMotionR3Factory::makeRotationalSymmetryFast(
    colvec const axis,
    double const theta,
    colvec const center
){
    RigidMotion rm = makeRotationalSymmetryFast(axis, theta);
    rm.setC((eye(3, 3)-rm.getA())*center);
    return rm;
}
