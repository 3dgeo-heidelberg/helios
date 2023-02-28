#pragma once

#include <MultiThreadKDTreeFactory.h>
#include <SAHKDTreeFactory.h>

using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Decorator for any SAH KDTree which provides support for multi thread
 *  KDTree building
 */
class MultiThreadSAHKDTreeFactory : public MultiThreadKDTreeFactory{
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
    template <class Archive>
    void serialize(Archive &ar, unsigned int const version){
        boost::serialization::void_cast_register<
            MultiThreadSAHKDTreeFactory,
            MultiThreadKDTreeFactory
        >();
        ar &boost::serialization::base_object<MultiThreadKDTreeFactory>(*this);
        ar &sahkdtf;
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
        shared_ptr<SimpleKDTreeFactory> const kdtf,
        size_t const numJobs=2
    );
    virtual ~MultiThreadSAHKDTreeFactory() = default;

};
