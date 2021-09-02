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
    // If there are no primitives, then KDTree will be null
    if(primitives.size() == 0) return nullptr;

    // Instantiate node which will be a root node if depth is 0
    KDTreeNode *node;
    if(depth > 0) node = new KDTreeNode();
    else node = new KDTreeNodeRoot();

    // Determine split axis and position
    int splitAxis;
    double splitPos;
    defineSplit(primitives, depth, splitAxis, splitPos);

    // Fill children's primitive lists
    vector<Primitive*> leftPrimitives, rightPrimitives;
    populateSplits(
        primitives, splitAxis, splitPos, leftPrimitives, rightPrimitives
    );

    // Build nodes from children's primitive list or make current one leaf
    buildChildrenNodes(
        primitives,
        splitAxis,
        depth,
        splitPos,
        leftPrimitives,
        rightPrimitives,
        node
    );

    // Return built node
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

void SimpleKDTreeFactory::defineSplit(
    vector<Primitive *> const &primitives,
    int const depth,
    int &splitAxis,
    double &splitPos
) const {
    // Find split axis
    splitAxis = depth % 3;

    // Sort faces along split axis:
    // ATTENTION: Sorting must happen BEFORE splitPos is computed as the median
    // Sort primitives along split axis:
    std::sort(
        primitives.begin(),
        primitives.end(),
        KDTreePrimitiveComparator(splitAxis)
    );

    // Compute split position from centroid of median primitive
    auto p = next(primitives.begin(), primitives.size()/2);
    splitPos = (*p)->getCentroid()[splitAxis];
}

void SimpleKDTreeFactory::populateSplits(
    vector<Primitive *> const &primitives,
    int const splitAxis,
    double const splitPos,
    vector<Primitive *> &leftPrimitives,
    vector<Primitive *> &rightPrimitives
) const {
    for(auto p : primitives){
        AABB *box = p->getAABB();
        if(box->getMin()[splitAxis] <= splitPos) leftPrimitives.push_back(p);
        if(box->getMax()[splitAxis] > splitPos) rightPrimitives.push_back(p);
    }
}

void SimpleKDTreeFactory::buildChildrenNodes(
    vector<Primitive *> const &primitives,
    int const splitAxis,
    int const depth,
    double const splitPos,
    vector<Primitive *> const &leftPrimitives,
    vector<Primitive *> const &rightPrimitives,
    KDTreeNode *node
) const {
    size_t const primsSize = primitives.size();
    if(
        leftPrimitives.size() != primsSize &&
        rightPrimitives.size() != primsSize
    ){
        node->splitAxis = splitAxis;
        node->splitPos = splitPos;
        if(!leftPrimitives.empty())
            node->left = buildRecursive(leftPrimitives, depth + 1);
        if(!rightPrimitives.empty())
            node->right = buildRecursive(rightPrimitives, depth + 1);
    }
    else {
        // Otherwise, make this node a leaf:
        node->splitAxis = -1;
        node->primitives = primitives;
    }
}
