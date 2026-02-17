#pragma once

#include <helios/adt/kdtree/MultiThreadKDTreeFactory.h>
#include <helios/adt/kdtree/SAHKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Decorator for any SAH KDTree which provides support for multi thread
 *  KDTree building
 */
class MultiThreadSAHKDTreeFactory : public MultiThreadKDTreeFactory
{
  using MultiThreadKDTreeFactory::kdtf;

private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a multi thread SAH KDTree factory to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the multi thread SAH K dimensional
   *  tree factory
   */
  template<class Archive>
  void serialize(Archive& ar, unsigned int const version)
  {
    boost::serialization::void_cast_register<MultiThreadSAHKDTreeFactory,
                                             MultiThreadKDTreeFactory>();
    ar& boost::serialization::base_object<MultiThreadKDTreeFactory>(*this);
    ar & sahkdtf;
  }

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Pointer to SAH KDTree as a casting of kdtf pointer
   * @see MultiThreadKDTreeFactory::kdtf
   */
  std::shared_ptr<SAHKDTreeFactory> sahkdtf;
  /**
   * @brief Mutex for the ILOT cache
   * @see SAHKDTreeFactory::buildChildrenNodes
   * @see SAHKDTreeFactory::internalizeILOT
   * @see SAHKDTreeFactory::toILOTCache
   * @see SAHKDTreeFactory::fromILOCache
   * @see SAHKDTreeFactory::getCacheT
   */
  boost::mutex ilotCacheMutex;
  /**
   * @brief Unique pointer to store unique lock for ILOT cache
   * @see MultiThreadSAHKDTreeFactory::ilotCacheMutex
   * @see SAHKDTreeFactory::_lockILOT
   * @see SAHKDTreeFactory::_unlockILOT
   */
  std::unique_ptr<boost::unique_lock<boost::mutex>> ilotCacheLock;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief MultiThreadSAHKDTreeFactory default constructor
   * @param kdtf The SAH factory to be used to build the KDTree
   */
  MultiThreadSAHKDTreeFactory(
    std::shared_ptr<SimpleKDTreeFactory> const kdtf,
    std::shared_ptr<SimpleKDTreeGeometricStrategy> const gs,
    std::size_t const numJobs = 2,
    std::size_t const geomJobs = 2);
  ~MultiThreadSAHKDTreeFactory() override = default;

  // ***  CLONE  *** //
  // *************** //
  /**
   * @see KDTreeFactory::clone
   */
  KDTreeFactory* clone() const override;
  /**
   *
   * @brief Assign attributes from MultiThreadSAHKDTreeFactory to its clone
   */
  void _clone(KDTreeFactory* kdtf) const override;
};
