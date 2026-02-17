#include <helios/adt/grove/KDGrove.h>
#include <helios/adt/grove/KDGroveSubject.h>
#include <helios/alg/raycast/GroveKDTreeRaycaster.h>
#include <helios/scene/dynamic/DynMovingObject.h>

// ***  BASIC DYNGROVE SUBJECT METHODS  *** //
// **************************************** //
void
KDGroveSubject::registerObserverGrove(
  std::shared_ptr<BasicDynGrove<GroveKDTreeRaycaster, DynMovingObject>>
    observer)
{
  registerObserverGrove(std::static_pointer_cast<KDGrove>(observer));
}
