#pragma once

#include <KDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for simple k-dimensional trees
 *
 * @see KDTreeFactory
 */
class SimpleKDTreeFactory : public KDTreeFactory{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a simple KDTree factory to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the simple KDTree factory
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        boost::serialization::void_cast_register<
            SimpleKDTreeFactory,
            KDTreeFactory
        >();
        ar &boost::serialization::base_object<SimpleKDTreeFactory>(*this);
    }

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief SimpleKDTreeFactory default constructor
     */
    SimpleKDTreeFactory() = default;
    virtual ~SimpleKDTreeFactory() = default;

    // ***  SIMPLE KDTREE FACTORY METHODS  *** //
    // *************************************** //
    /**
     * @brief Build a simple KDTree from given primitives
     * @param primitives Primitives to build simple KDTree splitting them
     * @return Pointer to root node of built simple KDTree
     */
    KDTreeNodeRoot* makeFromPrimitives(
        vector<Primitive *> const &primitives
    ) const override;

protected:
    // ***  BUILDING METHODS  *** //
    // ************************** //
    /**
     * @brief Recursively build a KDTree for given primitives
     * @param primitives Primitives to build KDTree splitting them
     * @param depth Current depth at build process. Useful for tracking
     *  recursion level
     * @return Built KDTree node
     */
    virtual KDTreeNode * buildRecursive(
        vector<Primitive*> primitives,
        int const depth
    ) const;
    /**
     * @brief Analyze KDTree computing its max depth and the minimum and
     *  maximum number of primitives considering all nodes
     * @param root Root node to compute stats for given KDTreeNodeRoot
     */
    virtual void computeKDTreeStats(KDTreeNodeRoot *root) const;


};