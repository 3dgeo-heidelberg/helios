#pragma once

#include <KDTreeNode.h>
#include <vector>
using std::vector;

/**
 * @brief Class representing the root node of a KDTree
 */
class KDTreeNodeRoot : public KDTreeNode {
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        boost::serialization::void_cast_register<KDTreeNodeRoot, KDTreeNode>();
        ar & boost::serialization::base_object<KDTreeNode>(*this);
        ar & stats_maxNumPrimsInLeaf;
        ar & stats_minNumPrimsInLeaf;
        ar & stats_maxDepthReached;
    }
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Maximum number of primitives considering all leaves
     */
    int stats_maxNumPrimsInLeaf;
    /**
     * @brief Minimum number of primitives considering all leaves
     */
    int stats_minNumPrimsInLeaf;
    /**
     * @brief Maximum depth of the KDTree
     */
    int stats_maxDepthReached;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for KDTree root node
     */
    KDTreeNodeRoot() :
        stats_maxNumPrimsInLeaf(0),
        stats_minNumPrimsInLeaf(std::numeric_limits<int>::max()),
        stats_maxDepthReached(0) {}

    ~KDTreeNodeRoot() override = default;

    // ***  CLASS METHODS  *** //
    // *********************** //
    /**
     * @brief KDTree build method
     * @param primitives Primitives to build KDTree splitting them
     * @return Pointer to root node of built KDTree
     */
    static KDTreeNodeRoot* build(const std::vector<Primitive*> & primitives);
};