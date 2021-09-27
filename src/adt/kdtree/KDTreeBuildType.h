#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Build type to wrap data required for recursive building of KDTree
 *  nodes when using a KDTreeFactory based thread pool
 * @see KDTreeFactoryThreadPool
 * @see KDTreeFactory
 */
class KDTreeBuildType{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The parent node
     * @see SimpleKDTreeFactory::buildRecursive
     */
    KDTreeNode *parent;
    /**
     * @brief True if given node is a left child, false otherwise. Notice root
     *  node is not left nor right, thus it must be false for root nodes.
     * @see SimpleKDTreeFactory::buildRecursive
     */
    bool left;
    /**
     * @brief Primitives to build KDTree node from
     * @see SimpleKDTreeFactory::buildRecursive
     */
    vector<Primitive *> primitives;
    /**
     * @brief Depth of node
     * @see SimpleKDTreeFactory::buildRecursive
     */
    int depth;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for KDTreeBuildType
     */
    KDTreeBuildType() :
        node(nullptr),
        left(false),
        primitives(0),
        depth(-1)
    {}

    /**
     * @brief Constructor for KDTreeBuildType with attributes as arguments
     * @see KDTreeBuildType::node
     * @see KDTreeBuildType::left
     * @see KDTreeBuildType::primitives
     * @see KDTreeBuildType::depth
     */
    KDTreeBuildType(
        KDTreeNode *node,
        bool const left,
        vector<Primitive *> &primitives,
        int const depth
    ) :
        node(node),
        left(left),
        primitives(primitives),
        depth(depth)
    {}

    virtual ~KDTreeBuildType() = default;


};