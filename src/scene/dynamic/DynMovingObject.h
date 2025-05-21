#pragma once

#include <deque>
#include <memory>

#include <adt/grove/KDGroveSubject.h>
#include <rigidmotion/RigidMotionEngine.h>
#include <scene/dynamic/DynMotion.h>
#include <scene/dynamic/DynMotionEngine.h>
#include <scene/dynamic/DynObject.h>
#include <sim/tools/VoidStepLoop.h>

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
class DynMovingObject
  : public DynObject
  , public KDGroveSubject
{
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
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::split_member(ar, *this, version);
  }
  /**
   * @brief Save a serialized dynamic moving object to a stream of bytes
   * @see DynMovingObject::serialize(Archive &, const unsigned int)
   * @see DynMovingObject::load(Archive &, const unsigned int)
   */
  template<typename Archive>
  void save(Archive& ar, const unsigned int version) const
  {
    boost::serialization::void_cast_register<DynMovingObject, DynObject>();
    ar& boost::serialization::base_object<DynObject>(*this);
    ar & positionMotionQueue;
    ar & normalMotionQueue;
    ar & dme;
    ar & observerStepLoop.getStepInterval();
  }
  /**
   * @brief Load a serialized dynamic moving object from a stream of bytes
   * @see DynMovingObject::serialize(Archive &, const unsigned int)
   * @see DynMovingObject::save(Archive &, const unsigned int)
   */
  template<typename Archive>
  void load(Archive& ar, const unsigned int version)
  {
    boost::serialization::void_cast_register<DynMovingObject, DynObject>();
    ar& boost::serialization::base_object<DynObject>(*this);
    ar & positionMotionQueue;
    ar & normalMotionQueue;
    ar & dme;
    int stepInterval;
    ar & stepInterval;
    observerStepLoop.setStepInterval(stepInterval);
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
  std::deque<std::shared_ptr<DynMotion>> positionMotionQueue;
  /**
   * @brief Queue of motions to be applied to the normal vector of each
   *  primitive
   *
   * The first motion in the queue is the first motion to be applied. The
   *  last motion in the queue is the last motion to be applied.
   *
   * @see DynMovingObject::positionMotionQueue
   */
  std::deque<std::shared_ptr<DynMotion>> normalMotionQueue;
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
   *  KDGrove.
   * @see DynMovingObject::kdGroveObserver
   */
  std::size_t groveSubjectId;
  /**
   * @brief Handle how many consecutive updates must elapse so the
   *  observer is notified.
   * @see DynMovingObject::kdGroveObserver
   * @see DynMovingObject::doObserverUpdate
   * @see DynMovingObject::getObserverStepInterval
   * @see DynMovingObject::setObserverStepInterval
   */
  VoidStepLoop<> observerStepLoop;
  /**
   * @brief The dynamic time step for the observer (i.e., dynamic KDT).
   *  It will be NaN when not given.
   * @see DynScene::dynTimeStep
   * @see DynObject::dynTimeStep
   */
  double observerDynTimeStep = std::numeric_limits<double>::quiet_NaN();

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @see DynObject::DynObject
   */
  DynMovingObject()
    : DynObject()
    , observerStepLoop(1, [&]() -> void { doObserverUpdate(); })
    , observerDynTimeStep(std::numeric_limits<double>::quiet_NaN())
  {
  }
  /**
   * @see DynObject::DynObject(ScenePart const &, bool const)
   */
  DynMovingObject(ScenePart const& sp, bool const shallowPrimitives = false)
    : DynObject(sp, shallowPrimitives)
    , kdGroveObserver(nullptr)
    , observerStepLoop(1, [&]() -> void { doObserverUpdate(); })
    , observerDynTimeStep(std::numeric_limits<double>::quiet_NaN())
  {
  }
  /**
   * @see DynObject::DynObject(string const)
   */
  DynMovingObject(std::string const id)
    : DynObject(id)
    , kdGroveObserver(nullptr)
    , observerStepLoop(1, [&]() -> void { doObserverUpdate(); })
    , observerDynTimeStep(std::numeric_limits<double>::quiet_NaN())
  {
  }
  /**
   * @see DynObject::DynObject(vector<Primitive *> const &)
   */
  DynMovingObject(std::vector<Primitive*> const& primitives)
    : DynObject(primitives)
    , kdGroveObserver(nullptr)
    , observerStepLoop(1, [&]() -> void { doObserverUpdate(); })
    , observerDynTimeStep(std::numeric_limits<double>::quiet_NaN())
  {
  }
  /**
   * @see DynObject::DynObject(string const, vector<Primitive *> const &)
   */
  DynMovingObject(std::string const id,
                  std::vector<Primitive*> const& primitives)
    : DynObject(id, primitives)
    , kdGroveObserver(nullptr)
    , observerStepLoop(1, [&]() -> void { doObserverUpdate(); })
    , observerDynTimeStep(std::numeric_limits<double>::quiet_NaN())
  {
  }
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
  bool doSimStep() override;
  /**
   * @brief Handle update notifications to the subscribed observer. It is,
   *  notify the observer that it has been updated by the dynamic moving
   *  object.
   * @see DynMovingObject::observerStepLoop
   * @see DynMovingObject::kdGroveObserver
   * @see DynMovingObject::getObserverStepInterval
   * @see DynMovingObject::setObserverStepInterval
   */
  virtual void doObserverUpdate();

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
    std::function<void(arma::mat const& X)> matrixToPrimitives,
    std::function<bool()> queueHasNext,
    std::function<std::shared_ptr<DynMotion>()> queueNext);

public:
  // ***  MOTION QUEUES METHODS  *** //
  // ******************************* //
  /**
   * @brief Push given dynamic motion to the position motion queue
   * @param dm Dynamic motion to be pushed to the position motion queue
   */
  inline void pushPositionMotion(std::shared_ptr<DynMotion> const dm)
  {
    positionMotionQueue.push_back(dm);
  }
  /**
   * @brief Retrieve the first dynamic motion in the position motion queue
   *
   * Notice this implies removing it from the queue.
   *
   * @return First dynamic motion in the position motion queue
   */
  inline std::shared_ptr<DynMotion> nextPositionMotion()
  {
    return _next(positionMotionQueue);
  }
  /**
   * @brief Remove all dynamic motions from the position motion queue
   */
  inline void clearPositionMotionQueue() { positionMotionQueue.clear(); }
  /**
   * @brief Check if position motion queue has a next dynamic motion or not
   * @return True if position motion queue has a next dynamic motion, false
   *  otherwise
   */
  inline bool positionMotionQueueHasNext() const
  {
    return !positionMotionQueue.empty();
  }
  /**
   * @brief Push given dynamic motion to the normal motion queue
   * @param dm Dynamic motion to be pushed to the normal motion queue
   */
  inline void pushNormalMotion(std::shared_ptr<DynMotion> const dm)
  {
    normalMotionQueue.push_back(dm);
  }
  /**
   * @brief Retrieve the first dynamic motion in the normal motion queue
   *
   * Notice this implies removing it from the queue.
   *
   * @return First dynamic motion in the normal motion queue
   */
  inline std::shared_ptr<DynMotion> nextNormalMotion()
  {
    return _next(normalMotionQueue);
  }
  /**
   * @brief Remove all dynamic motions from the normal motion queue
   */
  inline void clearNormalMotionQueue() { normalMotionQueue.clear(); }
  /**
   * @brief Check if normal motion queue has a next dynamic motion or not
   * @return True if normal motion queue has a next dynamic motion, false
   *  otherwise
   */
  inline bool normalMotionQueueHasNext() const
  {
    return !normalMotionQueue.empty();
  }

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
  std::shared_ptr<DynMotion> _next(
    std::deque<std::shared_ptr<DynMotion>>& deck);

public:
  // ***  GROVE SUBSCRIBER METHODS  *** //
  // ********************************** //
  /**
   * @brief Register given grove as a observer for the dynamic moving object
   * @param kdGroveObserver Grove to be registered as a observer
   * @see KDGroveSubject::registerObserverGrove
   */
  void registerObserverGrove(std::shared_ptr<KDGrove> kdGroveObserver) override;
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

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @see ScenePart::release
   */
  void release() override;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @see ScenePart::getType
   */
  ObjectType getType() const override { return ObjectType::DYN_MOVING_OBJECT; }
  /**
   * @brief Set the step interval between consecutive observer update
   *  notifications
   * @param stepInterval The new step interval between consecutive observer
   *  update notifications
   * @see DynMovingObject::observerStepLoop
   * @see DynMovingObject::kdGroveObserver
   * @see DynMovingObject::doObserverUpdate
   * @see DynMovingObject::getObserverStepInterval
   */
  inline void setObserverStepInterval(int const stepInterval)
  {
    observerStepLoop.setStepInterval(stepInterval);
  }
  /**
   * @brief Get the step interval between consecutive observer update
   *  notifications
   * @return The step interval between consecutive observer update
   *  notifications
   * @see DynMovingObject::observerStepLoop
   * @see DynMovingObject::kdGroveObserver
   * @see DynMovingObject::doObserverUpdate
   * @see DynMovingObject::setObserverStepInterval
   */
  inline int getObserverStepInterval() const
  {
    return observerStepLoop.getStepInterval();
  }

  /**
   * @brief Get the dynamic time step of the observer. Note it is not taken
   *  from the observer step loop, instead it represents a user-given
   *  parameter that must be used to configure the observerStepLoop.
   * @return The dynamic time step of the observer.
   * @see DynMovingObject::observerDynTimeStep
   * @see DynMovingObject::setObserverDynTimeStep
   * @see DynMovingObject::observerStepLoop
   */
  inline double getObserverDynTimeStep() const { return observerDynTimeStep; }
  /**
   * @brief Set the dynamic time step of the observer. Note it does not
   *  modify the observer step loop. The observerDynTimeStep attribute
   *  simply represents a user-given parameter. Setting it will not
   *  automatically update the observerStepLoop.
   * @param dynTimeStep The new dynamic time step for the observer.
   * @see DynMovingObject::observerDynTimeStep
   * @see DynMovingObject::getObserverDynTimeStep
   * @see DynMovingObject::observerStepLoop
   */
  inline void setObserverDynTimeStep(double const dynTimeStep)
  {
    observerDynTimeStep = dynTimeStep;
  }
};
