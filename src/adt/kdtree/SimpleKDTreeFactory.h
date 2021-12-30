#pragma once

#include <KDTreeFactory.h>
#include <SharedTaskSequencer.h>

#include <vector>
#include <functional>

class MultiThreadKDTreeFactory;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for simple k-dimensional trees
 *
 * @see KDTreeFactory
 */
class SimpleKDTreeFactory : public KDTreeFactory{
    // ***  FRIENDS  *** //
    // ***************** //
    friend class MultiThreadKDTreeFactory;

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
        ar &minSplitPrimitives;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The member function as attribute used to recursively build
     *  KDTree nodes. By default it will be assigned to the buildRecursive
     *  member function but it might be overridden by other implementations.
     *  For instance, to wrap the buildRecursive behavior to handle parallel
     *  building of KDTrees
     * @see SimpleKDTreeFactory::buildRecursive
     */
    std::function<KDTreeNode *(
        KDTreeNode *,
        bool const,
        vector<Primitive *> &,
        int const,
        int const
    )> _buildRecursive;

    /**
     * @brief How many primitives are required for a node to be splitted.
     *
     * If number of primitives is equal or greater than, then node might be
     *  splitted if necessary criterion is satisfied. Otherwise, no matter if
     *  other splitting criterion is satisfied, node will never be splitted.
     * It is mainly useful to prevent too deep KDTrees which might lead to
     *  consuming a high amount of memory without significant performance
     *  improvement.
     */
    size_t minSplitPrimitives;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief SimpleKDTreeFactory default constructor.
     *
     * The buildRecursive member function is assigned to the _buildRecursive
     *  function as member attribute
     */
    SimpleKDTreeFactory();
    virtual ~SimpleKDTreeFactory() = default;

    // ***  SIMPLE KDTREE FACTORY METHODS  *** //
    // *************************************** //
    /**
     * @brief Build a simple KDTree from given primitives
     * @param primitives Primitives to build simple KDTree splitting them
     * @return Pointer to root node of built simple KDTree
     */
    KDTreeNodeRoot * makeFromPrimitivesUnsafe(
        vector<Primitive *> &primitives
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
     * @param index The node index inside current depth. Each node can be
     *  univocally identified by the ordered pair \f$(d, i)\f$ where \f$d\f$
     *  stands for the depth level and \f$i\f$ for the index. The root node
     *  is identified by \f$(0, 0)\f$. Any left child node will be
     *  \f$(d+1, 2i)\f$ and any right child node will be \f$(d+1, 2i+1)\f$,
     *  where \f$d\f$ and \f$i\f$ are the depth and index for the parent node.
     *  In consequence, all left nodes will have an even index while all right
     *  nodes will have an odd one. However, notice that for performance
     *  reasons it could preferable to check the left flag argument, as it is
     *  faster than checking if index is even or odd.
     * @return Built KDTree node
     */
    virtual KDTreeNode * buildRecursive(
        KDTreeNode *parent,
        bool const left,
        vector<Primitive*> &primitives,
        int const depth,
        int const index
    );
    /**
     * @brief The recipe of the recursive building algorithm. It is meant
     *  to be used by the SimpleKDTreeFactory::buildRecursive but also by
     *  any alternative implementation which shares the same recipe (global
     *  logic) but changes the way some parts are computed. For instance, it
     *  is used by the MultiThreadKDTreeFactory to handle geometry-level
     *  parallelization
     *
     * @param f_computeNodeBoundaries The function to compute node boundaries.
     *  For the base case it is SimpleKDTreeFactory::buildRecursive
     * @param f_defineSplit The function to define split axis and position.
     *  For the base case it is SimpleKDTreeFactory::computeNodeBoundaries
     * @param f_populateSplits The function to populate left and right splits
     *  with corresponding primitives. For the base case it is
     *  SimpleKDTreeFactory::populateSplits
     * @param f_buildChildrenNodes The function to build the children nodes
     *  (left and right). For the base case it is
     *  SimpleKDTreeFactory::buildChildrenNodes
     *
     * @see SimpleKDTreeFactory::buildRecursive
     * @see SimpleKDTreeFactory::computeNodeBoundaries
     * @see SimpleKDTreeFactory::defineSplit
     * @see SimpleKDTreeFactory::populateSplits
     * @see SimpleKDTreeFactory::buildChildrenNodes
     * @see MultiThreadKDTreeFactory
     *
     * @return Built KDTree node
     */
    virtual KDTreeNode * buildRecursiveRecipe(
        KDTreeNode *parent,
        bool const left,
        vector<Primitive *> &primitives,
        int const depth,
        int const index,
        std::function<void(
            KDTreeNode *node,
            KDTreeNode *parent,
            bool const left,
            vector<Primitive *> const &primitives
        )> f_computeNodeBoundaries,
        std::function<void(
            KDTreeNode *node,
            KDTreeNode *parent,
            vector<Primitive *> &primitives,
            int const depth
        )> f_defineSplit,
        std::function<void(
            vector<Primitive *> const &primitives,
            int const splitAxis,
            double const splitPos,
            vector<Primitive *> &leftPrimitives,
            vector<Primitive *> &rightPrimitives
        )> f_populateSplits,
        std::function<void(
            KDTreeNode *node,
            KDTreeNode *parent,
            vector<Primitive *> const &primitives,
            int const depth,
            int const index,
            vector<Primitive *> &leftPrimitives,
            vector<Primitive *> &rightPrimitives
        )> f_buildChildrenNodes
    );
    /**
     * @brief Analyze KDTree computing its max depth and the minimum and
     *  maximum number of primitives considering all nodes
     * @param root Root node to compute stats for given KDTreeNodeRoot
     */
    virtual void computeKDTreeStats(KDTreeNodeRoot *root) const;
    /**
     * @brief Report KDTree stats of given root node at INFO logging level
     * @param root Root node which stats must be reported
     */
    virtual void reportKDTreeStats(
        KDTreeNodeRoot *root,
        vector<Primitive *> const &primitives
    ) const;

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
     * @see SimpleKDTreeFactory::onPopulateSplitsDigestPrimitive
     */
    virtual void populateSplits(
        vector<Primitive *> const &primitives,
        int const splitAxis,
        double const splitPos,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives
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
     * @param index Index of current node at current depth
     * @param leftPrimitives Primitives for left child node
     * @param rightPrimitives Primitives for right child node
     */
    virtual void buildChildrenNodes(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> const &primitives,
        int const depth,
        int const index,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives
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
     * @see SimpleKDTreeFactory::computeMinMaxSAHForChild
     * @see SimpleKDTreeFactory::onRootBoundariesDigestPrimitive
     * @see SimpleKDTreeFactory::onComputeNodeBoundariesCalcSAH
     */
    virtual void computeNodeBoundaries(
        KDTreeNode *node,
        KDTreeNode *parent,
        bool const left,
        vector<Primitive *> const &primitives
    ) const;

    /**
     * @brief Function to assist SimpleKDTreeFactory::populateSplits by
     *  providing the logic of digesting a primitive
     * @see SimpleKDTreeFactory::populateSplits
     */
    virtual void onPopulateSplitsDigestPrimitive(
        Primitive *p,
        int const splitAxis,
        double const splitPos,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives
    ) const;

    /**
     * @brief Function to assist SimpleKDTreeFactory::computeNodeBoundaries
     *  when computing surface area heuristic and minimum and maximum
     *  positions for child nodes
     * @param node The child node itself
     * @see SimpleKDTreeFactory::computeNodeBoundaries
     * @see SimpleKDTreeFactory::onRootBoundariesDigestPrimitive
     * @see SimpleKDTreeFactory::onComputeNodeBoundariesCalcSAH
     */
    virtual void computeMinMaxSAHForChild(
        KDTreeNode *node,
        KDTreeNode *parent,
        bool const left,
        vector<Primitive *> const &primitives
    ) const;

    /**
     * @brief Function to assist SimpleKDTreeFactory::computeNodeBoundaries by
     *  providing the logic of digesting a primitive
     * @param p Primitive to be digested
     * @see SimpleKDTreeFactory::computeNodeBoundaries
     * @see SimpleKDTreeFactory::computeMinMaxSAHForChild
     * @see SimpleKDTreeFactory::onComputeNodeBoundariesCalcSAH
     */
    virtual void onRootBoundariesDigestPrimitive(
        Primitive *primitive,
        double &ax,
        double &ay,
        double &az,
        double &bx,
        double &by,
        double &bz
    ) const;

    /**
     * @brief Function to assist SimpleKDTreeFactory::computeNodeBoundaries
     *  when computing the SAH for a node
     * @param[out] node Node which surface area and bound is going to be
     *  calculated (and setted)
     * @see SimpleKDTreeFactory::computeNodeBoundaries
     * @see SimpleKDTreeFactory::computeMinMaxSAHForChild
     * @see SimpleKDTreeFactory::onRootBoundariesDigestPrimitive
     */
    virtual void onComputeNodeBoundariesCalcSAH(
        KDTreeNode *node,
        double const ax,
        double const ay,
        double const az,
        double const bx,
        double const by,
        double const bz
    ) const;

    /**
     * @brief Check wheter the node must be splitted (true) or not (false)
     *  depending on its total primitives and the ones that would be assigned
     *  to left and right children.
     *
     * For a simple KDT a node must be splitted if there are enough primitives
     *  and not all of them are contained neither in left nor in right children
     *  nodes.
     *
     * @return True if node must be splitted, false otherwise
     * @see SimpleKDTreeFactory::minSplitPrimitives
     */
    virtual bool checkNodeMustSplit(
        vector<Primitive *> const &primitives,
        vector<Primitive *> const &leftPrimitives,
        vector<Primitive *> const &rightPrimitives
    ) const;

    /**
     * @brief Make given node a leaf one
     * @param[out] node Node to be made a leaf
     * @param primitives Primitives for the leaf node
     */
    virtual void makeLeaf(
        KDTreeNode *node,
        vector<Primitive *> const &primitives
    ) const;

    // ***  GEOMETRY LEVEL BUILDING  *** //
    // ********************************* //
    /**
     * @brief Geometry-level parallel version of the
     *  SimpleKDTreeFactory::defineSplit function
     * @param assignedThreads How many threads can be used to parallelize
     *  computations
     * @see SimpleKDTreeFactory::defineSplit
     */
    virtual void GEOM_defineSplit(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> &primitives,
        int const depth,
        int const assignedThreads
    ) const;
    /**
     * @brief Geometry-level parallel version of the
     *  SimpleKDTreeFactory::computeNodeBoundaries function
     * @param assignedThreads How many threads can be used to parallelize
     *  computations
     * @see SimpleKDTreeFactory::computeNodeBoundaries
     */
    virtual void GEOM_computeNodeBoundaries(
        KDTreeNode *node,
        KDTreeNode *parent,
        bool const left,
        vector<Primitive *> const &primitives,
        int assignedThreads
    );
    /**
     * @brief Geometry-level parallel version of the
     *  SimpleKDTreeFactory::populateSplits function
     * @param assignedThreads How many threads can be used to parallelize
     *  computations
     * @see SimpleKDTreeFactory::populateSplits
     */
    virtual void GEOM_populateSplits(
        vector<Primitive *> const &primitives,
        int const splitAxis,
        double const splitPos,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives,
        int assignedThreads
    ) const;
    /**
     * @brief Geometry-level parallel version of the
     *  SimpleKDTreeFactory::buildChildrenNodes function
     *
     * It is expected that this function is called at any depth before the
     *  last geometry-level depth. In consequence, the left node is delegated
     *  upon current thread while right node is delegated upon a new created
     *  thread for such purpose.
     *
     * @see SimpleKDTreeFactory::buildChildrenNodes
     * @return True if a new master thread has been created, false otherwise.
     *  This method will return when all pending work related to recursively
     *  building of children nodes and all its descendants has been completed
     */
    virtual void GEOM_buildChildrenNodes(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> const &primitives,
        int const depth,
        int const index,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives,
        std::shared_ptr<SharedTaskSequencer> masters
    );
};