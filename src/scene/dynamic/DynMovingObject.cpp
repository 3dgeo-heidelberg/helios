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
bool DynMovingObject::doStep(){
    // Flag to control whether the dynamic object has been modified or not
    bool modified = false;

    // Apply position rigid motions, if any
    if(positionMotionQueueHasNext()){
        arma::mat X = positionMatrixFromPrimitives();
        while(positionMotionQueueHasNext()){
            shared_ptr<RigidMotion> rm = nextPositionMotion();
            X = rme.apply(*rm, X);
        }
        updatePrimitivesPositionFromMatrix(X);
        modified = true;
    }

    // Apply normal rigid motions, if any
    if(normalMotionQueueHasNext()){
        arma::mat X = normalMatrixFromPrimitives();
        while(normalMotionQueueHasNext()){
            shared_ptr<RigidMotion> rm = nextNormalMotion();
            X = rme.apply(*rm, X);
        }
        updatePrimitivesNormalFromMatrix(X);
        modified = true;
    }

    // Return modifications control flag
    return modified;
}
