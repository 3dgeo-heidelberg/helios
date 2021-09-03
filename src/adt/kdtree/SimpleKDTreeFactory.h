#pragma once

#include <KDTreeFactory.h>

#include <vector>

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
        ar &boost::serialization::base_object<KDTreeFactory>(*this);
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
    KDTreeNodeRoot * makeFromPrimitives(
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
    /**
     * @brief Define the split axis and position for current node.
     *
     * The simple KDTree factory defines the split point as the position of
     *  the median which splits primitives along split axis into two nodes
     *  which tend to have the same number of primitives. To illustrate this,
     *  let \f$d\f$ be the depth of the node and \f$n\f$ be the space
     *  dimensionality. Therefore, the split axis can be defined as
     *  \f$a = d \mod n\f$. If \f$P=\left\{p_1, \ldots, p_n\right\}\f$ is the
     *  set of all primitives contained in the node where
     *  \f$\forall i, p_i=(p_{i1}, \ldots, p_{in})\f$, the split position
     *  \f$s\f$ can be used to define following sets:
     *  \f[
     *   \alpha = \left\{ p_i : p_{ia} \leq s \right\} \\
     *   \beta = \left\{ p_i : p_{ia} > s \right\}
     *  \f]
     *
     * But if \f$s\f$ is the median of primitives distribution along
     *  \f$a\f$-axis, then if there are no equal primitives it will be
     *  satisfied that \f$0 \leq |\alpha| - |\beta| \leq 1\f$
     *
     *
     * @param primitives Primitives of node which split must be defined
     * @param depth Depth of node which split must be defined
     * @param[out] splitAxis Store the index of split axis
     * @param[out] splitPos Store the split position
     */
    virtual void defineSplit(
        vector<Primitive *> &primitives,
        int const depth,
        int &splitAxis,
        double &splitPos
    ) const;
    /**
     * @brief Populate list of primitives for left and right splits from
     *  given primitives of node being splitted
     * @param primitives Primitives of node being splitted
     * @param splitAxis Index of axis defining the split
     * @param splitPos Position on given axis of the split point
     * @param[out] leftPrimitives Where primitives of left split must be stored
     * @param[out] rightPrimitives Where primitives of right split must be
     *  stored
     */
    virtual void populateSplits(
        vector<Primitive *> const &primitives,
        int const splitAxis,
        double const splitPos,
        vector<Primitive *> & leftPrimitives,
        vector<Primitive *> & rightPrimitives
    ) const;
    /**
     * @brief Build children nodes for given node. If no children nodes must
     *  be built, then the node is configured as a leaf node
     * @param primitives Primitives of the node itself
     * @param splitAxis Index of axis defining the split
     * @param depth Depth of current node
     * @param splitPos Position on given axis of the split point
     * @param leftPrimitives Primitives for left child node
     * @param rightPrimitives Primitives for right child node
     * @param[out] node Node which children will be built if possible and, if
     *  not, then will be configured as leaf node
     */
    virtual void buildChildrenNodes(
        vector<Primitive *> const &primitives,
        int const splitAxis,
        int const depth,
        double const splitPos,
        vector<Primitive *> const &leftPrimitives,
        vector<Primitive *> const &rightPrimitives,
        KDTreeNode *node
    ) const;


};