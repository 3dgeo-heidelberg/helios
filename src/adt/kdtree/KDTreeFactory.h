#pragma once

#include <KDTreeNode.h>
#include <KDTreeNodeRoot.h>

#include <vector>

using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class that must be extended by any class which provides factory
 *  methods for k-dimensional trees. Notice all KDTrees should be instantiated
 *  through corresponding factories.
 *
 * @see KDTreeNodeRoot
 */
class KDTreeFactory{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a KDTree factory to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the K dimensional tree factory
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {}
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief K dimensional tree factory default constructor
     */
    KDTreeFactory() = default;
    virtual ~KDTreeFactory() = default;

    // ***  K-DIMENSIONAL TREE FACTORY METHODS  *** //
    // **********************+********************* //
    /**
     * @brief Build a KDTree from given primitives
     * @param primitives Primitives to build KDTree splitting them
     * @return Pointer to root node of built KDTree
     */
    virtual KDTreeNodeRoot * makeFromPrimitivesUnsafe(
        vector<Primitive *> &primitives
    ) = 0;
    /**
     * @brief Safe wrapper from makeFromPrimitivesUnsafe which handles a copy
     *  to make from primitives by default. This function behavior might be
     *  overridden by any derived/child class. It is expected that any
     *  implementation of makeFromPrimitives provides a way to implement
     *  the makeFromPrimitivesUnsafe method without modifying vector of input
     *  primitives. Notice this does not mean primitives themselves cannot be
     *  modified, that depends on the type of KDTreeFactory. It only means
     *  that the vector itself will not be modified, for instance due to
     *  sorting purposes.
     * @see KDTreeFactory::makeFromPrimitivesUnsafe
     */
    virtual KDTreeNodeRoot * makeFromPrimitives(
        vector<Primitive *> const &primitives
    )
    {
        vector<Primitive *> _primitives = primitives; // Copy to work over
        return makeFromPrimitivesUnsafe(_primitives);
    };
};