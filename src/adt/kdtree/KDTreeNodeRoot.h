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
private:
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
        ar & stats_numInterior;
        ar & stats_numLeaves;
        ar & stats_totalCost;
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
    /**
     * @brief Number of interior nodes in the KDTree
     */
    int stats_numInterior;
    /**
     * @brief Number of leaf nodes in the KDTree
     */
    int stats_numLeaves;
    /**
     * @brief Total cost of three. It changes depending on tree building
     *  strategy. It might be NaN if tree building strategy is not based on
     *  computing any cost
     */
    double stats_totalCost;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Constructor for KDTree root node
     */
    KDTreeNodeRoot() :
        stats_maxNumPrimsInLeaf(0),
        stats_minNumPrimsInLeaf(std::numeric_limits<int>::max()),
        stats_maxDepthReached(0),
        stats_numInterior(0),
        stats_numLeaves(0),
        stats_totalCost(0.0)
    {}

    ~KDTreeNodeRoot() override = default;
};