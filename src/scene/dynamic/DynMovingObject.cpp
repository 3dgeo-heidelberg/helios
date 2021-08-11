#include <scene/dynamic/DynMovingObject.h>

// ***  MOTION QUEUES METHODS  *** //
// ******************************* //
shared_ptr<RigidMotion> DynMovingObject::_next(
    deque<shared_ptr<RigidMotion>> &deck
){
    shared_ptr<RigidMotion> rm = deck.front();
    deck.pop_front();
    return rm;
}

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
void DynMovingObject::doStep(){
    // Apply position rigid motions, if any
    if(positionMotionQueueHasNext()){
        arma::mat X = positionMatrixFromPrimitives();
        while(positionMotionQueueHasNext()){
            shared_ptr<RigidMotion> rm = nextPositionMotion();
            X = rme.apply(*rm, X);
        }
        updatePrimitivesPositionFromMatrix(X);
    }

    // Apply normal rigid motions, if any
    if(normalMotionQueueHasNext()){
        arma::mat X = normalMatrixFromPrimitives();
        while(normalMotionQueueHasNext()){
            shared_ptr<RigidMotion> rm = nextNormalMotion();
            X = rme.apply(*rm, X);
        }
        updatePrimitivesNormalFromMatrix(X);
    }
}
