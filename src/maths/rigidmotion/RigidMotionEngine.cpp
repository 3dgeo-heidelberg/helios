#include <maths/rigidmotion/RigidMotionEngine.h>
#include <maths/rigidmotion/RigidMotionException.h>

using namespace rigidmotion;

// ***  RIGID MOTION ENGINE METHODS  *** //
// ************************************* //
mat RigidMotionEngine::apply(
    RigidMotion const &f,
    mat const X
){
    mat Y = f.getA() * X;
    return f.getC() + Y.each_col();
}

colvec RigidMotionEngine::apply(
    RigidMotion const &f,
    colvec const X
){
    return f.getC() + f.getA() * X;
}

RigidMotion RigidMotionEngine::compose(
    RigidMotion const &f,
    RigidMotion const &g
){
    return f.compose(g);
}

mat RigidMotionEngine::computeFixedPoints(
    RigidMotion const &f,
    size_t &dim,
    bool safe
){
    // When safe mode is requested, check that indeed there are fixed points
    if(safe){
        if(!f.hasFixedPoints()){
            throw RigidMotionException(
                "RigidMotionEngine cannot compute fixed points for a rigid "
                "motion with no fixed points"
            );
        }
    }

    // Find space dimensionality and invariant dimensionality
    size_t n = f.getDimensionality();
    dim = f.findInvariantDimensionality();

    // The set of fixed points is the exact solution for the system (1 point)
    if(dim==0){
        return solve(eye(n, n)-f.getA(), f.getC());
    }

    // The set of fixed points is a lineal variety with dimension > 0
    colvec s;
    mat U, V;
    svd_econ(U, s, V, eye(n, n)-f.getA(), "left", "std");
    if(dim < n) return U.col(0);
    return U;
}
