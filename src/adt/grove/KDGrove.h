#pragma once

#include <BasicDynGrove.h>
#include <DynMovingObject.h>
#include <GroveKDTreeRaycaster.h>
#include <KDGroveStats.h>
#include <KDGroveSubject.h>

#include <memory>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Grove of KDTrees. It supports both static and dynamic KDTrees,
 *  handling each accordingly.
 * @see BasicDynGrove
 */
class KDGrove : public BasicDynGrove<GroveKDTreeRaycaster, DynMovingObject>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Stats of the KDGgrove
   * @see KDGroveStats
   */
  std::shared_ptr<KDGroveStats> stats;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for KDGrove
   * @param initTreesCapacity See the initTreesCapacity argument of
   *  BasicStaticGrove::BasicStaticGrove
   */
  KDGrove(size_t const initTreesCapacity = 1)
    : BasicDynGrove<GroveKDTreeRaycaster, DynMovingObject>(initTreesCapacity)
    , stats(nullptr)
  {
  }
  virtual ~KDGrove() = default;

  // ***  OBSERVER METHODS  *** //
  // ************************** //
  /**
   * @brief Workaround to redirect calls from
   *  KDGrove::addSubject(KDGroveSubject *, shared_ptr<GroveKDTreeRaycaster>)
   *  to BasicDynGrove::addSubject
   * @see BasicDynGrove::addSubject
   */
  inline void addSubject(KDGroveSubject* subject,
                         std::shared_ptr<GroveKDTreeRaycaster> tree)
  {
    BasicDynGrove::addSubject(
      (BasicDynGroveSubject<GroveKDTreeRaycaster, DynMovingObject>*)subject,
      tree);
  }
  /**
   * @brief Workaround to redirect calls from
   *  KDGrove::removeSubject(KDGroveSubject *) to
   *  BasicDynGrove::removeSubject
   * @see BasicDynGrove::removeSubject
   */
  inline void removeSubject(KDGroveSubject* subject)
  {
    BasicDynGrove::removeSubject(
      (BasicDynGroveSubject<GroveKDTreeRaycaster, DynMovingObject>*)subject);
  }

  // ***  KDGROVE METHODS  *** //
  // ************************* //
  /**
   * @brief Make a temporal clone of the KDGrove.
   *
   * The temporal clone preserves all static trees but holds its own copy
   *  for each dynamic tree. Therefore, source KDGrove can be updated, which
   *  means its dynamic trees can change, while the dynamic trees at the
   *  temporal clone are not affected by those changes.
   *
   * It is called a temporal clone because it clones the KDGrove at a certain
   *  time. Considering the KDGrove mutates over time. It is not a typical
   *  clone because it avoids cloning whatever components that do not change
   *  over time.
   *
   * Notice stats are not cloned. Only the main components, it is the trees.
   *  The subjects are neither updated. The temporal clone should not be
   *  updated. It must be understood simply as a way of holding the state of
   *  a KDGrove at a certain time, not as a new fully operating KDGrove.
   *
   * @return Temporal clone of the KDGrove
   * @see GroveKDTreeRaycaster::makeTemporalClone
   * @see KDGroveRaycaster::makeTemporalClone
   */
  virtual std::shared_ptr<KDGrove> makeTemporalClone() const;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the KDGrove stats
   * @return KDGrove stats
   */
  virtual std::shared_ptr<KDGroveStats> getStats() { return stats; }
  /**
   * @brief Set the KDGrove stats
   * @param stats New KDGrove stats
   */
  virtual void setStats(std::shared_ptr<KDGroveStats> stats)
  {
    this->stats = stats;
  }
};
