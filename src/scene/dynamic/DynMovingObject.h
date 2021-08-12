#pragma once

#include <memory>
#include <deque>

#include <scene/dynamic/DynObject.h>
#include <rigidmotion/RigidMotion.h>
#include <rigidmotion/RigidMotionEngine.h>

using std::shared_ptr;
using std::make_shared;
using std::deque;

using rigidmotion::RigidMotionEngine;
using rigidmotion::RigidMotion;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Implementation of a dynamic object which supports rigid motion
 *
 * This dynamic object handles any rigid motion in \f$\mathbb{R}^2\f$ and
 *  \f$\mathbb{R}^{3}\f$.
 *
 * @see DynObject
 * @see rigidmotion::RigidMotion
 * @see rigidmotion::RigidMotionEngine
 * @see rigidmotion::RigidMotionR2Factory
 * @see rigidmotion::RigidMotionR3Factory
 */
class DynMovingObject : public DynObject{
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
    deque<shared_ptr<RigidMotion>> positionMotionQueue;
    /**
     * @brief Queue of motions to be applied to the normal vector of each
     *  primitive
     *
     * The first motion in the queue is the first motion to be applied. The
     *  last motion in the queue is the last motion to be applied.
     *
     * @see DynMovingObject::positionMotionQueue
     */
    deque<shared_ptr<RigidMotion>> normalMotionQueue;
    /**
     * @brief The rigid motion engine to apply rigid motions
     * @see rigidmotion::RigidMotionEngine
     */
    RigidMotionEngine rme;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @see DynObject::DynObject
     */
    DynMovingObject() = default;
    /**
     * @see DynObject::DynObject(string const)
     */
    DynMovingObject(string const id) : DynObject(id) {}
    /**
     * @see DynObject::DynObject(vector<Primitive *> const &)
     */
    DynMovingObject(vector<Primitive *> const &primitives) :
        DynObject(primitives)
    {}
    /**
     * @see DynObject::DynObject(string const, vector<Primitive *> const &)
     */
    DynMovingObject(string const id, vector<Primitive *> const &primitives) :
        DynObject(id, primitives)
    {}
    virtual ~DynMovingObject() = default;

    // ***  DYNAMIC BEHAVIOR  *** //
    // ************************** //
    /**
     * @brief Rigid motion behavior implementation
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
     *  the rigid motion. Then, \f$C'\f$ and \f$A'\f$ are analogous to \f$C\f$
     *  and \f$A\f$ but coming from the normal motion queue instead of the
     *  position motion queue.
     *
     * @see DynObject::doStep
     * @see rigidmotion::RigidMotion
     * @see DynMovingObject::positionMotionQueue
     * @see DynMovingObject::normalMotionQueue
     */
    bool doStep() override;

    // ***  MOTION QUEUES METHODS  *** //
    // ******************************* //
    /**
     * @brief Push given rigid motion to the position motion queue
     * @param rm Rigid motion to be pushed to the position motion queue
     */
    inline void pushPositionMotion(shared_ptr<RigidMotion> const rm)
    {positionMotionQueue.push_back(rm);}
    /**
     * @brief Retrieve the first in rigid motion in the position motion queue
     *
     * Notice this implies removing it from the queue.
     *
     * @return First rigid motion in the position motion queue
     */
    inline shared_ptr<RigidMotion> nextPositionMotion()
    {return _next(positionMotionQueue);}
    /**
     * @brief Remove all rigid motions from the position motion queue
     */
    inline void clearPositionMotionQueue()
    {positionMotionQueue.clear();}
    /**
     * @brief Check if position motion queue has a next rigid motion or not
     * @return True if position motion queue has a next rigid motion, false
     *  otherwise
     */
    inline bool positionMotionQueueHasNext() const
    {return !positionMotionQueue.empty();}
    /**
     * @brief Push given rigid motion to the normal motion queue
     * @param rm Rigid motion to be pushed to the normal motion queue
     */
    inline void pushNormalMotion(shared_ptr<RigidMotion> const rm)
    {normalMotionQueue.push_back(rm);}
    /**
     * @brief Retrieve the first rigid motion in the normal motion queue
     *
     * Notice this implies removing it from the queue.
     *
     * @return First rigid motion in the normal motion queue
     */
    inline shared_ptr<RigidMotion> nextNormalMotion()
    {return _next(normalMotionQueue);}
    /**
     * @brief Remove all rigid motions from the normal motion queue
     */
    inline void clearNormalMotionQueue()
    {normalMotionQueue.clear();}
    /**
     * @brief Check if normal motion queue has a next rigid motion or not
     * @return True if normal motion queue has a next rigid motion, false
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
     * @return First rigid motion in the given queue
     */
    shared_ptr<RigidMotion> _next(deque<shared_ptr<RigidMotion>> &deck);
};