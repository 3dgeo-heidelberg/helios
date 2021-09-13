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
    ) override;

protected:
    // ***  BUILDING METHODS  *** //
    // ************************** //
    /**
     * @brief Recursively build a KDTree for given primitives
     * @param parent The parent node if any. For root nodes, it must be a
     *  nullptr
     * @param left True if given node is a left child, false otherwise.
     *  If the node is a root node, it should be false. If node is not a root
     *  node and left is true, it means it is a left child. If node is not a
     *  root node and left is false, it means it is a right child
     * @param primitives Primitives to build KDTree splitting them
     * @param depth Current depth at build process. Useful for tracking
     *  recursion level
     * @return Built KDTree node
     */
    virtual KDTreeNode * buildRecursive(
        KDTreeNode *parent,
        bool const left,
        vector<Primitive*> primitives,
        int const depth
    ) ;
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
     * @param[out] node Node which split axis and split position are going to
     *  be defined
     * @param parent The parent node if any. For root nodes, it will be a
     *  nullptr
     * @param primitives Primitives of node which split must be defined
     * @param depth Depth of node which split must be defined
     */
    virtual void defineSplit(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> &primitives,
        int const depth
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
     *
     * @param[out] node Node which children will be built if possible and, if
     *  not, then will be configured as leaf node
     * @param parent The parent node if any. For root nodes, it will be a
     *  nullptr
     * @param primitives Primitives of the node itself
     * @param depth Depth of current node
     * @param leftPrimitives Primitives for left child node
     * @param rightPrimitives Primitives for right child node
     */
    virtual void buildChildrenNodes(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> const &primitives,
        int const depth,
        vector<Primitive *> const &leftPrimitives,
        vector<Primitive *> const &rightPrimitives
    );

    // ***  BUILDING UTILS  *** //
    // ************************ //
    /**
     * @brief Compute min and max position and surface area of bounding cuboid
     *  for given node
     *
     * Surface area for a cuboid can be computed considering \f$l_i\f$ is the
     *  length of \f$i-th\f$ axis.
     *
     * For instance, surface area of root node \f$R\f$ in \f$\mathbb{R}^2\f$
     *  can be computed as follows:
     *
     * \f[
     *  S_A(R) = l_x l_y
     * \f]
     *
     *
     * Analogously, the surface area of root node \f$R\f$ in \f$\mathbb{R}^3\f$
     *  can be computed as follows:
     *
     * \f[
     *  S_A(R) = 2(l_x l_y + l_x l_z + l_y l_z)
     * \f]
     *
     * The boundaries for the root node are taken from min and max vertex of
     *  given set of primitives. Surface area and min and max boundaries for
     *  children nodes are just taken proportionally to \f$r\f$ where \f$p\f$
     *  is the split position and \f$a\f$ and \f$b\f$ are the min and max
     *  positions respectively:
     *
     * \f[
     *  r = \frac{p-a}{b-a}
     * \f]
     *
     * For the sake of understanding, surface area for children nodes is
     *  explained in detail. For this purpose, notice that \f$p \in [a, b]\f$.
     *  Now lets define left and right children surface area respectively,
     *  namely \f$S_A(L_b)\f$ and \f$S_A(R_b)\f$ where \f$S_A(P)\f$ is the
     *  surface area of the parent node itself:
     *
     * \f[
     * \left\{\begin{array}{lllll}
     *  S_A(L_b) &=& r S_A(P) &=& \frac{p-a}{b-a} S_A(P) \\
     *  S_A(R_b) &=& (1-r) S_A(P) &=& \left(1 - \frac{p-a}{b-a}\right) S_A(P)
     * \end{array}\right.
     * \f]
     *
     * @param node Root node which surface area must be computed
     * @param parent The parent node if any. For root nodes, it will be a
     *  nullptr
     * @param left True if given node is a left child, false otherwise.
     *  If the node is a root node, it should be false. If node is not a root
     *  node and left is true, it means it is a left child. If node is not a
     *  root node and left is false, it means it is a right child
     * @param primitives Vector of primitives inside given root node
     */
    virtual void computeNodeBoundaries(
        KDTreeNode *node,
        KDTreeNode *parent,
        bool const left,
        vector<Primitive *> const &primitives
    ) const;

};