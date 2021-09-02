#include <SimpleKDTreeFactory.h>
#include <logging.hpp>
#include <KDTreePrimitiveComparator.h>
#include <IBinaryTreeNode.h>
#include <BinaryTreeDepthIterator.h>

// ***  SIMPLE KDTREE FACTORY METHODS  *** //
// *************************************** //
KDTreeNodeRoot* SimpleKDTreeFactory::makeFromPrimitives(
    vector<Primitive *> const &primitives
) const {
    KDTreeNodeRoot *root = (KDTreeNodeRoot *) buildRecursive(primitives, 0);
    std::stringstream ss;
    if(root == nullptr){
        /*
         * NOTICE building a null KDTree is not necessarily a bug.
         * It is a natural process that might arise for instance when upgrading
         *  from static scene to dynamic scene in the XmlSceneLoader.
         * This message is not reported at INFO level because it is only
         *  relevant for debugging purposes.
         */
        ss  << "Null KDTree with no primitives was built";
        logging::DEBUG(ss.str());
    }
    else{
        computeKDTreeStats(root);
        ss  << "KDTree (num. primitives " << primitives.size() << ") :\n\t"
            << "Max. # primitives in leaf: "
            << root->stats_maxNumPrimsInLeaf << "\n\t"
            << "Min. # primitives in leaf: "
            << root->stats_minNumPrimsInLeaf << "\n\t"
            << "Max. depth reached: : "
            << root->stats_maxDepthReached;
        logging::INFO(ss.str());
    }
    return root;
}

KDTreeNode * SimpleKDTreeFactory::buildRecursive(
    vector<Primitive*> primitives,
    int const depth
) const {
    size_t primsSize = primitives.size();
    if (primsSize == 0) {
        return nullptr;
    }

    KDTreeNode *node;
    if(depth > 0) node = new KDTreeNode();
    else node = new KDTreeNodeRoot();

    // TODO 5: Implement surface area heuristics?
    int splitAxis = depth % 3;

    // Sort faces along split axis:
    // ATTENTION: Sorting must happen BEFORE splitPos is computed as the median

    // Sort primitives along split axis:
    std::sort(
        primitives.begin(),
        primitives.end(),
        KDTreePrimitiveComparator(splitAxis)
    );

    // Compute split position:
    auto p = next(primitives.begin(), primsSize / 2);
    double splitPos = (*p)->getCentroid()[splitAxis];

    // ########## BEGIN Fill children's primitive lists ##########

    vector<Primitive*> sublist_left;
    vector<Primitive*> sublist_right;

    for (auto p : primitives) {
        AABB* box = p->getAABB();

        if (box->getMin()[splitAxis] <= splitPos) {
            sublist_left.push_back(p);
        }

        if (box->getMax()[splitAxis] > splitPos) {
            sublist_right.push_back(p);
        }
    }

    if(sublist_left.size() != primsSize && sublist_right.size() != primsSize){
        node->splitAxis = splitAxis;
        node->splitPos = splitPos;
        if(!sublist_left.empty())
            node->left = buildRecursive(sublist_left, depth + 1);
        if(!sublist_right.empty())
            node->right = buildRecursive(sublist_right, depth + 1);
    }
    else {
        // Otherwise, make this node a leaf:
        node->splitAxis = -1;
        node->primitives = primitives;
    }

    return node;
}

void SimpleKDTreeFactory::computeKDTreeStats(KDTreeNodeRoot *root) const{
    BinaryTreeDepthIterator<KDTreeNode> btdi(root, 0);
    int maxDepth = 0;
    int maxNumPrimsInLeaf = 0;
    int minNumPrimsInLeaf = std::numeric_limits<int>::max();
    while(btdi.hasNext()){
        IterableTreeNode<IBinaryTreeNode> node = btdi.next();
        if(node.getDepth() > maxDepth) maxDepth = node.getDepth();
        KDTreeNode * const kdtNode = static_cast<KDTreeNode *>(node.getNode());
        if(kdtNode->isLeafNode()){
            int const numPrims = kdtNode->primitives.size();
            if(numPrims > maxNumPrimsInLeaf) maxNumPrimsInLeaf = numPrims;
            if(numPrims < minNumPrimsInLeaf) minNumPrimsInLeaf = numPrims;
        }
    }
    root->stats_maxNumPrimsInLeaf = maxNumPrimsInLeaf;
    root->stats_minNumPrimsInLeaf = minNumPrimsInLeaf;
    root->stats_maxDepthReached = maxDepth;
}
