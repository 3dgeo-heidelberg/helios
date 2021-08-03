#include <maths/rigidmotion/RigidMotionEngine.h>

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
