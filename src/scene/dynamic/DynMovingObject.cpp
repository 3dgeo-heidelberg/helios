#include <scene/dynamic/DynMovingObject.h>

#include <functional>

// ***  MOTION QUEUES METHODS  *** //
// ******************************* //
shared_ptr<DynMotion> DynMovingObject::_next(
    deque<shared_ptr<DynMotion>> &deck
){
    shared_ptr<DynMotion> dm = deck.front();
    deck.pop_front();
    return dm;
}

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
bool DynMovingObject::doStep(){
    // Flag to control whether the dynamic object has been modified or not
    bool modified = false;

    // Apply position dynamic motions, if any
    modified |= applyDynMotionQueue(
        [=]()->arma::mat{
            return this->positionMatrixFromPrimitives();
        },
        [=](arma::mat const &X)->void{
            this->updatePrimitivesPositionFromMatrix(X);
        },
        [=]()->bool{
            return this->positionMotionQueueHasNext();
        },
        [=]()->shared_ptr<DynMotion>{
            return this->nextPositionMotion();
        }
    );

    // Apply normal dynamic motions, if any
    modified |= applyDynMotionQueue(
        [=]()->arma::mat{
            return this->normalMatrixFromPrimitives();
        },
        [=](arma::mat const &X)->void{
            this->updatePrimitivesNormalFromMatrix(X);
        },
        [=]()->bool{
            return this->normalMotionQueueHasNext();
        },
        [=]()->shared_ptr<DynMotion>{
            return this->nextNormalMotion();
        }
    );

    // Return modifications control flag
    return modified;
}
bool DynMovingObject::applyDynMotionQueue(
    std::function<arma::mat()> matrixFromPrimitives,
    std::function<void(arma::mat const &X)> matrixToPrimitives,
    std::function<bool()> queueHasNext,
    std::function<shared_ptr<DynMotion>()> queueNext
){
    // Flag to control whether the dynamic object has been modified or not
    bool modified = false;

    // Apply dynamic motions from queue, if any
    if(queueHasNext()){
        arma::mat X = matrixFromPrimitives();
        shared_ptr<DynMotion> dm = nullptr;
        while(queueHasNext()){
            if(dm == nullptr) dm = queueNext();
            else{
                dm = make_shared<DynMotion>(
                    dme.compose(*queueNext(), *dm, *this)
                );
            }
        }
        X = dme.apply(*dm, X, *this);
        matrixToPrimitives(X);
        modified = true;
    }

    // Return modifications control flag
    return modified;
}
