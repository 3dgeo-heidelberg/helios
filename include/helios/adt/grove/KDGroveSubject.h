#pragma once

#include <helios/adt/grove/BasicDynGroveSubject.h>

#include <memory>

class KDGrove;
class GroveKDTreeRaycaster;
class DynMovingObject;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface defining the behaviors that must be supported by any
 *  object that can notify to a KDGrove
 * @see KDGrove
 */
class KDGroveSubject
  : public BasicDynGroveSubject<GroveKDTreeRaycaster, DynMovingObject>
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  virtual ~KDGroveSubject() = default;

  // ***  BASIC DYNGROVE SUBJECT METHODS  *** //
  // **************************************** //
  /**
   * @brief Workaround to redirect calls from
   *  BasicDynGroveObjectSubject::registerObserverGrove to
   *  KDGroveSubject::registerObserverGrove(std::shared_ptr<KDGroveSubject>)
   *  method
   * @see BasicDynGroveSubject::registerObserverGrove
   * @see KDGroveSubject::registerObserverGrove
   */
  void registerObserverGrove(
    std::shared_ptr<BasicDynGrove<GroveKDTreeRaycaster, DynMovingObject>>
      observer) override;

  // ***  KDGROVE SUBJECT METHODS  *** //
  // ********************************* //
  /**
   * @brief Register given KDGrove as a observer with respect to the subject
   * @see BasicDynGroveSubject::registerObserverGrove
   */
  virtual void registerObserverGrove(std::shared_ptr<KDGrove> observer) = 0;
};
