#pragma once

#include <KDTreeFactory.h>

using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Decorator for any KDTree factory which provides support for multi
 *  thread KDTree building
 */
class MultiThreadKDTreeFactory : public KDTreeFactory{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a multi thread KDTree factory to a stream of bytes
     * @tparam Archive Type of rendeering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the K dimensional tree factory
     */
    template <class Archive>
    void serialize(Archive &ar, unsigned int const version){
        boost::serialization::void_cast_register<
            MultiThreadKDTreeFactory,
            KDTreeFactory
        >();

        ar &boost::serialization::base_object<KDTreeFactory>(*this);
        ar &kdtf;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The KDTreeFactory to be used to build partial trees
     */
    shared_ptr<KDTreeFactory> kdtf;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief MultiThreadKDTreeFactory default constructor
     * @param kdtf The factory to be used to build the KDTree
     */
    MultiThreadKDTreeFactory(shared_ptr<KDTreeFactory> kdtf) :
        KDTreeFactory(),
        kdtf(kdtf)
    {}
    virtual ~MultiThreadKDTreeFactory() = default;

    // ***  MULTI THREAD KDTREE FACTORY METHODS  *** //
    // ********************************************* //
    /**
     * @brief Build a KDTree which type depends on current KDTree factory
     *  (MultiThreadKDTreeFactory::kdtf) on a multi thread basis
     * @see MultiThreadKDTreeFactory::kdtf
     * @see KDTreeFactory::makeFromPrimitivesUnsafe
     */
    KDTreeNodeRoot * makeFromPrimitivesUnsafe(
        vector<Primitive *> &primitives
    ) override;
};