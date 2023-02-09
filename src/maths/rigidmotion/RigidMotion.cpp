#include <rigidmotion/RigidMotion.h>
#include <rigidmotion/RigidMotionException.h>
#include <sstream>

using rigidmotion::RigidMotion;
using rigidmotion::RigidMotionException;
using std::stringstream;

// ***  CONSTANTS  *** //
// ******************* //
double const RigidMotion::eps = 0.000000001;

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

RigidMotion::SuperType RigidMotion::findSuperType() const{
    size_t const n = getDimensionality();
    size_t const r = rank(eye(n, n)-A, eps);
    bool rankMismatchesDimensionality = false;
    if(n==2){
        if(r==0) return SuperType::R2_BASE;
        else if(r==1) return SuperType::R2_REFLECTION;
        else if(r==2) return SuperType::R2_ROTATION;
        else rankMismatchesDimensionality = true;
    }
    else if(n==3){
        if(r==0) return SuperType::R3_BASE;
        else if(r==1) return SuperType::R3_REFLECTION;
        else if(r==2) return SuperType::R3_ROTATION;
        else if(r==3) return SuperType::R3_ROTATIONAL_SYMMETRY;
        else rankMismatchesDimensionality = true;
    }
    else{
        stringstream ss;
        ss  << "Rigid motion super type could not be determined because "
            << "unexpected dimensionality: " << n;
        throw RigidMotionException(ss.str());
    }

    if(rankMismatchesDimensionality){
        stringstream ss;
        ss  << "Rigid motion super type could not be determined because "
        << "unexpected rank " << r << " for dimensionality " << n;
        throw RigidMotionException(ss.str());
    }

    // Below code is never reached but it is necessary to prevent certain
    // compilers to raise non-void function with no return warnings
    throw RigidMotionException(
        "Unexpected behavior for rigidmotion::RigidMotion::findSuperType "
        "method"
    );
}

bool RigidMotion::hasFixedPoints() const{
    size_t n = getDimensionality();
    mat coef = (eye(n, n) - A);  // coef = I-A
    mat sys = join_rows(coef, C); // sys = (I-A | C)
    return rank(coef, eps) == rank(sys, eps);
}

RigidMotion::Type RigidMotion::findType() const{
    RigidMotion::SuperType st = findSuperType();
    bool fixedPoints = hasFixedPoints();
    switch (st) {
        default:
            throw RigidMotionException(
                "Unexpected type configuration for rigid motion"
            );
        case R2_BASE:
            if(fixedPoints) return IDENTITY_R2;
            return TRANSLATION_R2;
        case R2_REFLECTION:
            if(fixedPoints) return REFLECTION_R2;
            return GLIDE_REFLECTION_R2;
        case R2_ROTATION:
            if(fixedPoints) return ROTATION_R2;
            throw RigidMotionException(
                "Unexpected type configuration for R2 rotation rigid motion"
            );
        case R3_BASE:
            if(fixedPoints) return IDENTITY_R3;
            return TRANSLATION_R3;
        case R3_REFLECTION:
            if(fixedPoints) return REFLECTION_R3;
            return GLIDE_REFLECTION_R3;
        case R3_ROTATION:
            if(fixedPoints) return ROTATION_R3;
            return HELICAL_R3;
        case R3_ROTATIONAL_SYMMETRY:
            if(fixedPoints) return ROTATIONAL_SYMMETRY_R3;
            throw RigidMotionException(
                "Unexpected type configuration for R3 rotational symmetry "
                "rigid motion"
            );
    }
}

size_t RigidMotion::findInvariantDimensionality() const{
    size_t n = getDimensionality();
    return n-rank(eye(n, n)-A, eps);
}
