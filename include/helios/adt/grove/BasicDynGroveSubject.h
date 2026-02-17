#pragma once

#include <helios/adt/grove/BasicDynGrove.h>

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface defining the behaviors that must be supported by any
 *  object that can notify to a basic dynamic grove
 * @tparam Tree The type of the tree to be handled
 * @tparam Subject The type of the subject that can be observed
 * @see BasicDynGrove
 */
template<typename Tree, typename Subject>
class BasicDynGroveSubject
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  virtual ~BasicDynGroveSubject() = default;

  // ***  BASIC DYNGROVE SUBJECT METHODS  *** //
  // **************************************** //
  /**
   * @brief Register given grove as a observer with respect to the subject
   * @param observer Basic dynamic grove to be registered as a observer
   */
  virtual void registerObserverGrove(
    std::shared_ptr<BasicDynGrove<Tree, Subject>> observer) = 0;
  /**
   * @brief Unregister current basic dynamic grove observer
   */
  virtual void unregisterObserverGrove() = 0;
  /**
   * @brief Set the subject identifier to be given one
   * @param id New identifier for the subject
   */
  virtual void setGroveSubjectId(std::size_t const id) = 0;
  /**
   * @brief Get the subject identifier
   * @return Subject identifier
   */
  virtual std::size_t getGroveSubjectId() = 0;
};
