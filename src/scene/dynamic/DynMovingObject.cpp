#include <scene/dynamic/DynMovingObject.h>

#include <functional>

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
    modified |= applyRigidMotionQueue(
        [=]()->arma::mat{
            return this->positionMatrixFromPrimitives();
        },
        [=](arma::mat const &X)->void{
            this->updatePrimitivesPositionFromMatrix(X);
        },
        [=]()->bool{
            return this->positionMotionQueueHasNext();
        },
        [=]()->shared_ptr<RigidMotion>{
            return this->nextPositionMotion();
        }
    );

    // Apply normal rigid motions, if any
    modified |= applyRigidMotionQueue(
        [=]()->arma::mat{
            return this->normalMatrixFromPrimitives();
        },
        [=](arma::mat const &X)->void{
            this->updatePrimitivesNormalFromMatrix(X);
        },
        [=]()->bool{
            return this->normalMotionQueueHasNext();
        },
        [=]()->shared_ptr<RigidMotion>{
            return this->nextNormalMotion();
        }
    );

    // Return modifications control flag
    return modified;
}
bool DynMovingObject::applyRigidMotionQueue(
    std::function<arma::mat()> matrixFromPrimitives,
    std::function<void(arma::mat const &X)> matrixToPrimitives,
    std::function<bool()> queueHasNext,
    std::function<shared_ptr<RigidMotion>()> queueNext
){
    // Flag to control whether the dynamic object has been modified or not
    bool modified = false;

    // Apply rigid motions from queue, if any
    if(queueHasNext()){
        arma::mat X = matrixFromPrimitives();
        shared_ptr<RigidMotion> rm = nullptr;
        while(queueHasNext()){
            if(rm == nullptr) rm = queueNext();
            else{
                rm = make_shared<RigidMotion>(rme.compose(*queueNext(), *rm));
            }
        }
        X = rme.apply(*rm, X);
        matrixToPrimitives(X);
        modified = true;
    }

    // Return modifications control flag
    return modified;
}
