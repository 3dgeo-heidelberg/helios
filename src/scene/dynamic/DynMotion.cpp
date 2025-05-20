#include <DynMotion.h>

// ***  NORMALS UTILS  *** //
// *********************** //
bool
DynMotion::checkModifiesNormal() const
{
  rigidmotion::RigidMotion::SuperType stype = findSuperType();
  return stype == rigidmotion::RigidMotion::SuperType::R2_REFLECTION ||
         stype == rigidmotion::RigidMotion::SuperType::R2_ROTATION ||
         stype == rigidmotion::RigidMotion::SuperType::R3_REFLECTION ||
         stype == rigidmotion::RigidMotion::SuperType::R3_ROTATION ||
         stype == rigidmotion::RigidMotion::SuperType::R3_ROTATIONAL_SYMMETRY;
}

DynMotion
DynMotion::makeNormalCounterpart() const
{
  DynMotion dm(arma::zeros(getDimensionality()), getA());
  dm.setNormalMode(true);
  dm.setSelfMode(selfMode);
  return dm;
}
