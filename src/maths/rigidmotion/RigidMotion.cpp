#include <rigidmotion/RigidMotion.h>

using rigidmotion::RigidMotion;

// ***  RIGID MOTION METHODS  *** //
// ****************************** //
RigidMotion RigidMotion::compose(RigidMotion const rm) const {
    RigidMotion comp;
    comp.C = C + A * rm.C;
    comp.A = A * rm.A;
    return comp;
}

void RigidMotion::composeInPlace(RigidMotion const rm){
    *this = compose(rm);
}
