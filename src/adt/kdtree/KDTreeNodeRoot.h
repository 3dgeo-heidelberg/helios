#pragma once

#include <KDTreeNode.h>
#include <vector>
using std::vector;

/**
 * @brief Class representing the root node of a KDTree
 *
 * @see KDTreeNode
 */
class KDTreeNodeRoot : public KDTreeNode {
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a KDTreeNodeRoot to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the KDTreeNodeRoot
     */
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version){
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
};