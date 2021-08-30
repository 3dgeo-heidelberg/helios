#include <DynMotion.h>

// ***  NORMALS UTILS  *** //
// *********************** //
bool DynMotion::checkModifiesNormal() const{
    RigidMotion::SuperType stype = findSuperType();
    return
        stype == RigidMotion::SuperType::R2_REFLECTION ||
        stype == RigidMotion::SuperType::R2_ROTATION ||
        stype == RigidMotion::SuperType::R3_REFLECTION ||
        stype == RigidMotion::SuperType::R3_ROTATION ||
        stype == RigidMotion::SuperType::R3_ROTATIONAL_SYMMETRY
    ;
}

DynMotion DynMotion::makeNormalCounterpart() const{
    DynMotion dm(zeros(getDimensionality()), getA());
    dm.setNormalMode(true);
    dm.setSelfMode(selfMode);
    return dm;
}
