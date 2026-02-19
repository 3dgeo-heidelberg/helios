#pragma once

#include <AxisSAHKDTreeFactory.h>
#include <FastSAHKDTreeFactory.h>
#include <KDGrove.h>
#include <KDTreeFactory.h>
#include <MultiThreadKDTreeFactory.h>
#include <MultiThreadSAHKDTreeFactory.h>
#include <SAHKDTreeFactory.h>
#include <SimpleKDTreeFactory.h>

#include <memory>
#include <vector>

class KDGroveFactory
{
private:
  // *********************** //

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The KDTree factory used to build trees composing the grove
   */
  std::shared_ptr<KDTreeFactory> kdtf;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief K dimensional grove factory default constructor
   */
  KDGroveFactory(std::shared_ptr<KDTreeFactory> kdtf)
    : kdtf(kdtf)
  {
  }
  virtual ~KDGroveFactory() = default;

  // ***  K-DIMENSIONAL GROVE FACTORY METHODS  *** //
  // ********************************************* //
  /**
   * @brief Bulid a KDGrove from given scene parts
   * @param parts Scene parts to build KDGrove from
   * @param mergeNonMoving If true, all primitives from non moving objects
   *  will be merged to build a single static KDTree. If false, then one
   *  KDTree will be built for each static object.
   * @param computeKDGroveStats If true, KDGrove stats will be computed. If
   *  false, they will not
   * @param reportKDGroveStats If true, KDGrove stats will be reported. If
   *  false, they will not
   * @param computeKDTreeStats  If true, stats for each KDTree will be
   *  computed. If false, they will not
   * @param reportKDTreeStats If true, stats for each KDTree will be
   *  reported. If false, they will not
   * @return Build KDGrove
   * @see KDGrove
   * @see KDGroveFactory::makeFull
   * @see KDGroveFactory::makeMergeNonMoving
   */
  virtual std::shared_ptr<KDGrove> makeFromSceneParts(
    std::vector<std::shared_ptr<ScenePart>> parts,
    bool const mergeNonMoving = false,
    bool const safe = false,
    bool const computeKDGroveStats = false,
    bool const reportKDGroveStats = false,
    bool const computeKDTreeStats = false,
    bool const reportKDTreeStats = false);

  // ***  STATISTICS METHODS  *** //
  // **************************** //
  /**
   * @brief Method to handle the update of KDGrove stats after all KDTrees
   *  and the KDGrove itself have been created
   * @param kdgrove The KDGrove which stats must be computed
   * @param buildingTimes The building time for each tree inside the KDGrove
   * @param treePrimitives The number of primitives for each tree inside
   *  the KDGrove
   */
  void handleKDGroveStats(std::shared_ptr<KDGrove> kdgrove,
                          std::vector<double>& buildingTimes,
                          std::vector<int>& treePrimitives);

protected:
  // ***  UTIL BUILDING METHODS  *** //
  // ******************************* //
  /**
   * @brief The common implementation of building a KDGrove. It handles
   *  the common building process for both full and merge non-moving modes.
   * @see KDGroveFactory::makeFromSceneParts
   * @see KDGroveFactory::makeFull
   * @see KDGroveFactory::makeMergeNonMoving
   */
  virtual std::shared_ptr<KDGrove> makeCommon(
    std::vector<std::shared_ptr<ScenePart>> parts,
    bool const safe,
    bool const computeKDGroveStats,
    bool const reportKDGroveStats,
    bool const computeKDTreeStats,
    bool const reportKDTreeStats);
  /**
   * @brief Build a KDGrove on a KDTree per ScenePart basis
   * @see KDGroveFactory::makeFromSceneParts
   */
  virtual std::shared_ptr<KDGrove> makeFull(
    std::vector<std::shared_ptr<ScenePart>> parts,
    bool const safe,
    bool const computeKDGroveStats,
    bool const reportKDGroveStats,
    bool const computeKDTreeStats,
    bool const reportKDTreeStats);
  /**
   * @brief Build a KDGrove where all non moving scene parts are merged
   *  to build a single KDTree
   * @see KDGroveFactory::makeFromSceneParts
   */
  virtual std::shared_ptr<KDGrove> makeMergeNonMoving(
    std::vector<std::shared_ptr<ScenePart>> parts,
    bool const safe,
    bool const computeKDGroveStats,
    bool const reportKDGroveStats,
    bool const computeKDTreeStats,
    bool const reportKDTreeStats);

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the KDTreeFactory used by the KDGroveFactory
   * @return KDTreeFactory used by the KDGroveFactory
   * @see KDGroveFactory::kdtf
   */
  inline std::shared_ptr<KDTreeFactory> getKdtf() const { return kdtf; }
  /**
   * @brief Set the KDTreeFactory to be used by the KDGroveFactory
   * @param kdtf New KDTreeFactory to be used by the KDGroveFactory
   * @see KDGroveFactory::kdtf
   */
  inline void setKdtf(std::shared_ptr<KDTreeFactory> kdtf)
  {
    this->kdtf = kdtf;
  }
};
