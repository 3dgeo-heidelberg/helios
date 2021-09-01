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
    virtual KDTreeNodeRoot* makeFromPrimitives(
        vector<Primitive *> const &primitives
    ) const = 0;
};