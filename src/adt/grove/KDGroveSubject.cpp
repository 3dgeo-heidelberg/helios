#include <KDGroveSubject.h>
#include <KDGrove.h>
#include <GroveKDTreeRaycaster.h>
#include <DynMovingObject.h>

// ***  BASIC DYNGROVE SUBJECT METHODS  *** //
// **************************************** //
void KDGroveSubject::registerObserverGrove(
    std::shared_ptr<BasicDynGrove<GroveKDTreeRaycaster, DynMovingObject>>
    observer
){
    registerObserverGrove(std::static_pointer_cast<KDGrove>(observer));
}
