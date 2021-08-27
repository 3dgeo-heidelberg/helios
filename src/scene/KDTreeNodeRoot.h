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
    /**
     * @brief Serialize a KDTreeNodeRoot to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the KDTreeNodeRoot
     */
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version){
        std::cout << "Exporting KDTreeRoodNode (1) ..." << std::endl; // TODO Remove
        boost::serialization::void_cast_register<KDTreeNodeRoot, KDTreeNode>();
        std::cout << "Exporting KDTreeRoodNode (2) ..." << std::endl; // TODO Remove
        ar & boost::serialization::base_object<KDTreeNode>(*this);
        std::cout << "Exporting KDTreeRoodNode (3) ..." << std::endl; // TODO Remove
        ar & stats_maxNumPrimsInLeaf;
        std::cout << "Exporting KDTreeRoodNode (4) ..." << std::endl; // TODO Remove
        ar & stats_minNumPrimsInLeaf;
        std::cout << "Exporting KDTreeRoodNode (5) ..." << std::endl; // TODO Remove
        ar & stats_maxDepthReached;
        std::cout << "Exported KDTreeRoodNode!" << std::endl; // TODO Remove
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