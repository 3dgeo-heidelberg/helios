#pragma once

#include <memory>
#include <deque>

#include <scene/dynamic/DynObject.h>
#include <scene/dynamic/DynMotion.h>
#include <rigidmotion/RigidMotionEngine.h>
#include <scene/dynamic/DynMotionEngine.h>
#include <adt/grove/KDGroveSubject.h>

using std::shared_ptr;
using std::make_shared;
using std::deque;

using rigidmotion::RigidMotionEngine;

class KDGrove;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Implementation of a dynamic object which supports dynamic motions
 *  (extended rigid motions)
 *
 * This dynamic object handles any rigid motion in \f$\mathbb{R}^2\f$ and
 *  \f$\mathbb{R}^{3}\f$ and also extended functionalities coming from
 *  dynamic motion extra features, if any
 *
 * @see DynObject
 * @see rigidmotion::RigidMotion
 * @see DynMotion
 * @see rigidmotion::RigidMotionEngine
 * @see rigidmotion::RigidMotionR2Factory
 * @see rigidmotion::RigidMotionR3Factory
 */
class DynMovingObject : public DynObject, public KDGroveSubject{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a dynamic moving object to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the dynamic moving object
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        boost::serialization::void_cast_register<DynMovingObject, DynObject>();
        ar &boost::serialization::base_object<DynObject>(*this);
        ar &positionMotionQueue;
        ar &normalMotionQueue;
        ar &dme;
    }
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Queue of motions to be applied to the position of each primitive
     *
     * The first motion in the queue is the first motion to be applied. The
     *  last motion in the queue is the last motion to be applied.
     *
     * @see DynMovingObject::normalMotionQueue
     */
    deque<shared_ptr<DynMotion>> positionMotionQueue;
    /**
     * @brief Queue of motions to be applied to the normal vector of each
     *  primitive
     *
     * The first motion in the queue is the first motion to be applied. The
     *  last motion in the queue is the last motion to be applied.
     *
     * @see DynMovingObject::positionMotionQueue
     */
    deque<shared_ptr<DynMotion>> normalMotionQueue;
    /**
     * @brief The dynamic motion engine to apply dynamic motions
     * @see rigidmotion::RigidMotionEngine
     * @see DynMotionEngine
     */
    DynMotionEngine dme;
    /**
     * @brief The observer grove to which the dynamic moving object is
     *  registered
     * @see BasicDynGrove
     * @see KDGrove
     */
    std::shared_ptr<KDGrove> kdGroveObserver;
    /**
     * @brief The identifier of the dynamic moving object as subject of a
     *  KDGrove
     * @see DynMovingObject::kdGroveObserver
     */
    size_t groveSubjectId;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @see DynObject::DynObject
     */
    DynMovingObject() = default;
    /**
     * @see DynObject::DynObject(ScenePart const &)
     */
    DynMovingObject(ScenePart const &sp) :
        DynObject(sp),
        kdGroveObserver(nullptr)
    {}
    /**
     * @see DynObject::DynObject(string const)
     */
    DynMovingObject(string const id) :
        DynObject(id),
        kdGroveObserver(nullptr)
    {}
    /**
     * @see DynObject::DynObject(vector<Primitive *> const &)
     */
    DynMovingObject(vector<Primitive *> const &primitives) :
        DynObject(primitives),
        kdGroveObserver(nullptr)
    {}
    /**
     * @see DynObject::DynObject(string const, vector<Primitive *> const &)
     */
    DynMovingObject(string const id, vector<Primitive *> const &primitives) :
        DynObject(id, primitives),
        kdGroveObserver(nullptr)
    {}
    virtual ~DynMovingObject() = default;

    // ***  DYNAMIC BEHAVIOR  *** //
    // ************************** //
    /**
     * @brief Dynamic motion behavior implementation
     *
     * Let a moving dynamic object \f$\mathcal{D}\f$ be defined by matrices
     *  \f$P_{n \times m}\f$ defining the position and \f$U_{n \times m}\f$
     *  defining the normals. So the dynamic object is composed by \f$m\f$
     *  vertices and normals in a \f$n\f$-dimensional space and its update
     *  function would be as follows:
     *
     * \f[
     *  \mathcal{D}_{t+1} = f\left(\mathcal{D}_{t}\right) =
     *  \left\{\begin{array}{lll}
     *      P_{t+1} &=& C + AP_{t} \\
     *      U_{t+1} &=& C' + A'P_{t}
     *  \end{array}\right.
     * \f]
     *
     * Where \f$C_{n \times 1}\f$ is the transposition column vector and
     *  \f$A_{n \times n}\f$ is the fixed origin transformation matrix defining
     *  the rigid motion at the core of the dynamic motion. Then, \f$C'\f$ and
     *  \f$A'\f$ are analogous to \f$C\f$ and \f$A\f$ but coming from the
     *  normal motion queue instead of the position motion queue.
     *
     * @return True if the dynamic object was modified, false otherwise
     * @see DynObject::doStep
     * @see rigidmotion::RigidMotion
     * @see DynMotion
     * @see DynMovingObject::positionMotionQueue
     * @see DynMovingObject::normalMotionQueue
     */
    bool doStep() override;

protected:
    /**
     * @brief Method to assist dynamic motions computation.
     *
     * It provides the abstract logic to compute dynamic motions in an
     *  efficient way. Functions to extract dynamic motion queue elements and to
     *  read and update primitives must be given as input arguments.
     *
     * @param matrixFromPrimitives Function to get a matrix from primitives
     *  composing the dynamic object
     * @param matrixToPrimitives Function to update primitives from a given
     *  matrix
     * @param queueHasNext Function to check whether the queue has dynamic
     *  motions left (true) or not (false)
     * @param queueNext Function to obtain the next dynamic motion from a queue
     * @return True if modifications occurred, false otherwise
     */
    bool applyDynMotionQueue(
        std::function<arma::mat()> matrixFromPrimitives,
        std::function<void(arma::mat const &X)> matrixToPrimitives,
        std::function<bool()> queueHasNext,
        std::function<shared_ptr<DynMotion>()> queueNext
    );

public:
    // ***  MOTION QUEUES METHODS  *** //
    // ******************************* //
    /**
     * @brief Push given dynamic motion to the position motion queue
     * @param dm Dynamic motion to be pushed to the position motion queue
     */
    inline void pushPositionMotion(shared_ptr<DynMotion> const dm)
    {positionMotionQueue.push_back(dm);}
    /**
     * @brief Retrieve the first dynamic motion in the position motion queue
     *
     * Notice this implies removing it from the queue.
     *
     * @return First dynamic motion in the position motion queue
     */
    inline shared_ptr<DynMotion> nextPositionMotion()
    {return _next(positionMotionQueue);}
    /**
     * @brief Remove all dynamic motions from the position motion queue
     */
    inline void clearPositionMotionQueue()
    {positionMotionQueue.clear();}
    /**
     * @brief Check if position motion queue has a next dynamic motion or not
     * @return True if position motion queue has a next dynamic motion, false
     *  otherwise
     */
    inline bool positionMotionQueueHasNext() const
    {return !positionMotionQueue.empty();}
    /**
     * @brief Push given dynamic motion to the normal motion queue
     * @param dm Dynamic motion to be pushed to the normal motion queue
     */
    inline void pushNormalMotion(shared_ptr<DynMotion> const dm)
    {normalMotionQueue.push_back(dm);}
    /**
     * @brief Retrieve the first dynamic motion in the normal motion queue
     *
     * Notice this implies removing it from the queue.
     *
     * @return First dynamic motion in the normal motion queue
     */
    inline shared_ptr<DynMotion> nextNormalMotion()
    {return _next(normalMotionQueue);}
    /**
     * @brief Remove all dynamic motions from the normal motion queue
     */
    inline void clearNormalMotionQueue()
    {normalMotionQueue.clear();}
    /**
     * @brief Check if normal motion queue has a next dynamic motion or not
     * @return True if normal motion queue has a next dynamic motion, false
     *  otherwise
     */
    inline bool normalMotionQueueHasNext() const
    {return !normalMotionQueue.empty();}

protected:
    /**
     * @brief Assist next operation over given queue
     *
     * Notice next operation implies removing the retrieved element from the
     *  queue
     *
     * @param deck Queue to retrieve next from
     * @return First dynamic motion in the given queue
     */
    shared_ptr<DynMotion> _next(deque<shared_ptr<DynMotion>> &deck);

public:
    // ***  GROVE SUBSCRIBER METHODS  *** //
    // ********************************** //
    /**
     * @brief Register given grove as a observer for the dynamic moving object
     * @param kdGroveObserver Grove to be registered as a observer
     * @see KDGroveSubject::registerObserverGrove
     */
    void registerObserverGrove(shared_ptr<KDGrove> kdGroveObserver) override;
    /**
     * @brief Unregister current grove observer
     * @see KDGroveSubject::unregisterObserverGrove
     */
    void unregisterObserverGrove() override;
    /**
     * @see BasicDynGroveSubject::setGroveSubjectId
     */
    void setGroveSubjectId(std::size_t const id) override;
    /**
     * @see BasicDynGroveSubject::getGroveSubjectId
     */
    std::size_t getGroveSubjectId() override;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see ScenePart::getType
     */
    ObjectType getType() override {return ObjectType::DYN_MOVING_OBJECT;}
};