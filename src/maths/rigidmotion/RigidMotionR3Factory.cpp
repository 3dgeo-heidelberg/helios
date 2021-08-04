#include <rigidmotion/RigidMotionR3Factory.h>

using namespace rigidmotion;

// ***  CONSTANTS  *** //
// ******************* //
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
        alpha[0] = M_SQRT2;
        alpha[1] = M_SQRT2;
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
