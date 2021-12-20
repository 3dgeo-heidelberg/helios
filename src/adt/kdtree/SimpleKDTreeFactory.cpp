#include <SimpleKDTreeFactory.h>
#include <logging.hpp>
#include <KDTreePrimitiveComparator.h>
#include <IBinaryTreeNode.h>
#include <BinaryTreeDepthIterator.h>
#include <SharedTaskSequencer.h>
#include <SimpleKDTreePopulateSplitsSubTask.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SimpleKDTreeFactory::SimpleKDTreeFactory() : minSplitPrimitives(5) {
    /*
     * It is safe to call virtual function here.
     * Calling virtual functions on constructor means derived overrides will
     *  be ignored. But in this constructor what must be addressed is that
     *  by default the _buildRecursive member must point to the class
     *  buildRecursive itself.
     * All derived classes are responsible of setting the _buildRecursive
     *  member after calling parent constructor
     */
    _buildRecursive =
        [&](
            KDTreeNode *parent,
            bool const left,
            vector<Primitive *> &primitives,
            int const depth,
            int const index
        ) -> KDTreeNode * {
            return this->buildRecursive(
                parent, left, primitives, depth, index
            );
        }
    ;
}

// ***  SIMPLE KDTREE FACTORY METHODS  *** //
// *************************************** //
KDTreeNodeRoot* SimpleKDTreeFactory::makeFromPrimitivesUnsafe(
    vector<Primitive *> &primitives
) {
    // Build the KDTree using a modifiable copy of primitives pointers vector
    KDTreeNodeRoot *root = (KDTreeNodeRoot *) _buildRecursive(
        nullptr,        // Parent node
        false,          // Node is not left child, because it is root not child
        primitives,     // Primitives to be contained inside the KDTree
        0,              // Starting depth level (must be 0 for root node)
        0               // Starting index at depth 0 (must be 0 for root node)
    );
    if(root == nullptr){
        /*
         * NOTICE building a null KDTree is not necessarily a bug.
         * It is a natural process that might arise for instance when upgrading
         *  from static scene to dynamic scene in the XmlSceneLoader.
         * This message is not reported at INFO level because it is only
         *  relevant for debugging purposes.
         */
        std::stringstream ss;
        ss  << "Null KDTree with no primitives was built";
        logging::DEBUG(ss.str());
    }
    else{
        computeKDTreeStats(root);
        reportKDTreeStats(root, primitives);
        if(buildLightNodes) lighten(root);
    }
    return root;
}

KDTreeNode * SimpleKDTreeFactory::buildRecursive(
    KDTreeNode *parent,
    bool const left,
    vector<Primitive*> &primitives,
    int const depth,
    int const index
) {
    // If there are no primitives, then KDTree will be null
    if(primitives.empty()) return nullptr;

    // Instantiate node which will be a root node if depth is 0
    KDTreeNode *node;
    if(depth > 0) node = new KDTreeNode();
    else node = new KDTreeNodeRoot();
    computeNodeBoundaries(node, parent, left, primitives);

    // Determine split axis and position
    defineSplit(node, parent, primitives, depth);

    // Fill children's primitive lists
    vector<Primitive*> leftPrimitives, rightPrimitives;
    populateSplits(
        primitives,
        node->splitAxis,
        node->splitPos,
        leftPrimitives,
        rightPrimitives
    );

    // Build nodes from children's primitive list or make current one leaf
    buildChildrenNodes(
        node,
        parent,
        primitives,
        depth,
        index,
        leftPrimitives,
        rightPrimitives
    );

    // Return built node
    return node;
}

void SimpleKDTreeFactory::computeKDTreeStats(KDTreeNodeRoot *root) const{
    BinaryTreeDepthIterator<KDTreeNode> btdi(root, 0);
    int maxDepth = 0;
    int maxNumPrimsInLeaf = 0;
    int minNumPrimsInLeaf = std::numeric_limits<int>::max();
    int numInterior = 0;
    int numLeaves = 0;
    while(btdi.hasNext()){
        IterableTreeNode<IBinaryTreeNode> node = btdi.next();
        if(node.getDepth() > maxDepth) maxDepth = node.getDepth();
        KDTreeNode * const kdtNode = static_cast<KDTreeNode *>(node.getNode());
        if(kdtNode->isLeafNode()){
            int const numPrims = kdtNode->primitives->size();
            if(numPrims > maxNumPrimsInLeaf) maxNumPrimsInLeaf = numPrims;
            if(numPrims < minNumPrimsInLeaf) minNumPrimsInLeaf = numPrims;
            ++numLeaves;
        }
        else ++numInterior;
    }
    root->stats_maxNumPrimsInLeaf = maxNumPrimsInLeaf;
    root->stats_minNumPrimsInLeaf = minNumPrimsInLeaf;
    root->stats_maxDepthReached = maxDepth;
    root->stats_numInterior = numInterior;
    root->stats_numLeaves = numLeaves;
    root->stats_totalCost = 0.0;
}

void SimpleKDTreeFactory::reportKDTreeStats(
    KDTreeNodeRoot *root,
    vector<Primitive *> const &primitives
) const {
    std::stringstream ss;
    ss  << "KDTree (num. primitives " << primitives.size() << ") :\n\t"
        << "Max. # primitives in leaf: "
        << root->stats_maxNumPrimsInLeaf << "\n\t"
        << "Min. # primitives in leaf: "
        << root->stats_minNumPrimsInLeaf << "\n\t"
        << "Max. depth reached: "
        << root->stats_maxDepthReached << "\n\t"
        << "KDTree axis-aligned surface area: "
        << root->surfaceArea << "\n\t"
        << "Interior nodes: "
        << root->stats_numInterior << "\n\t"
        << "Leaf nodes: "
        << root->stats_numLeaves << "\n\t"
        << "Total tree cost: "
        << root->stats_totalCost;
    logging::INFO(ss.str());
}

void SimpleKDTreeFactory::defineSplit(
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> &primitives,
    int const depth
) const {
    // Find split axis
    node->splitAxis = depth % 3;

    // Sort faces along split axis:
    // ATTENTION: Sorting must happen BEFORE splitPos is computed as the median
    // Sort primitives along split axis:
    std::sort(
        primitives.begin(),
        primitives.end(),
        KDTreePrimitiveComparator(node->splitAxis)
    );

    // Compute split position from centroid of median primitive
    auto p = next(primitives.begin(), primitives.size()/2);
    node->splitPos = (*p)->getCentroid()[node->splitAxis];
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
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> const &primitives,
    int const depth,
    int const index,
    vector<Primitive *> &leftPrimitives,
    vector<Primitive *> &rightPrimitives
){
    size_t const primsSize = primitives.size();
    if(
        primsSize >= minSplitPrimitives &&
        leftPrimitives.size() != primsSize &&
        rightPrimitives.size() != primsSize
    ){ // If there are primitives on both partitions, binary split the node
        if(!leftPrimitives.empty()){
            setChild(node->left, _buildRecursive(
                node, true, leftPrimitives, depth + 1, 2*index
            ));
        }
        if(!rightPrimitives.empty()){
            setChild(node->right, _buildRecursive(
                node, false, rightPrimitives, depth + 1, 2*index + 1
            ));
        }
    }
    else {
        // Otherwise, make this node a leaf:
        node->splitAxis = -1;
        node->primitives = std::make_shared<vector<Primitive *>>(primitives);
    }
}

// ***  BUILDING UTILS  *** //
// ************************ //
void SimpleKDTreeFactory::computeNodeBoundaries(
    KDTreeNode *node,
    KDTreeNode *parent,
    bool const left,
    vector<Primitive *> const &primitives
) const {
    // Find surface area and minimum and maximum positions for root node
    if(parent == nullptr){
        double ax = std::numeric_limits<double>::max();
        double ay=ax, az=ax;
        double bx = std::numeric_limits<double>::lowest();
        double by=bx, bz=bx;
        for(Primitive *primitive : primitives){
            Vertex * vertices = primitive->getVertices();
            size_t const m = primitive->getNumVertices();
            for(size_t i = 0 ; i < m ; ++i){
                Vertex *vertex = vertices+i;
                if(vertex->getX() < ax) ax = vertex->getX();
                if(vertex->getY() < ay) ay = vertex->getY();
                if(vertex->getZ() < az) az = vertex->getZ();
                if(vertex->getX() > bx) bx = vertex->getX();
                if(vertex->getY() > by) by = vertex->getY();
                if(vertex->getZ() > bz) bz = vertex->getZ();
            }
        }

        // Compute surface area
        double const lx = bx-ax;
        double const ly = by-ay;
        double const lz = bz-az;
        node->surfaceArea = 2*(lx*ly + lx*lz + ly*lz);
        node->bound = AABB(ax, ay, az, bx, by, bz);
    }
    // Find surface area and minimum and maximum positions for child node
    else{
        double const a = parent->bound.getMin()[parent->splitAxis];
        double const b = parent->bound.getMax()[parent->splitAxis];
        double const p = parent->splitPos;
        double const ratio = (p - a) / (b - a);
        double ax = parent->bound.getMin()[0];
        double ay = parent->bound.getMin()[1];
        double az = parent->bound.getMin()[2];
        double bx = parent->bound.getMax()[0];
        double by = parent->bound.getMax()[1];
        double bz = parent->bound.getMax()[2];
        if(left){ // Left child node
            if(parent->splitAxis == 0) bx = ax + ratio * (bx-ax);
            if(parent->splitAxis == 1) by = ay + ratio * (by-ay);
            if(parent->splitAxis == 2) bz = az + ratio * (bz-az);
            node->surfaceArea = ratio * parent->surfaceArea;
            node->bound = AABB(ax, ay, az, bx, by, bz);
        }
        else{ // Right child node
            if(parent->splitAxis == 0) ax = ax + ratio * (bx-ax);
            if(parent->splitAxis == 1) ay = ay + ratio * (by-ay);
            if(parent->splitAxis == 2) az = az + ratio * (bz-az);
            node->surfaceArea = (1.0-ratio) * parent->surfaceArea;
            node->bound = AABB(ax, ay, az, bx, by, bz);
        }
    }
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
void SimpleKDTreeFactory::GEOM_populateSplits(
    vector<Primitive *> const &primitives,
    int const splitAxis,
    double const splitPos,
    vector<Primitive *> &leftPrimitives,
    vector<Primitive *> &rightPrimitives,
    int const assignedThreads
) const {
    // Distribute workload
    size_t const numPrimitives = primitives.size();
    size_t const chunkSize = numPrimitives / ((size_t)assignedThreads);
    vector<vector<Primitive *>> leftPrims(assignedThreads);
    vector<vector<Primitive *>> rightPrims(assignedThreads);
    int const extraThreads = assignedThreads - 1;
    std::shared_ptr<SharedTaskSequencer> stSequencer = \
        std::make_shared<SharedTaskSequencer>(extraThreads);
    for(int i = 0 ; i < extraThreads ; ++i){
        stSequencer->start(std::make_shared<SimpleKDTreePopulateSplitsSubTask>(
            stSequencer,
            primitives,
            splitAxis,
            splitPos,
            leftPrims[i],
            rightPrims[i],
            i*chunkSize,
            (i+1)*chunkSize
        ));
    }
    SimpleKDTreePopulateSplitsSubTask(
        nullptr,
        primitives,
        splitAxis,
        splitPos,
        leftPrims[extraThreads],
        rightPrims[extraThreads],
        extraThreads*chunkSize,
        primitives.size()
    )();

    // Wait until workload has been consumed
    stSequencer->joinAll();

    // Reduce left and right primitives into single vector
    for(int i = 0 ; i < assignedThreads ; i++){
        leftPrimitives.insert(
            leftPrimitives.end(),
            leftPrims[i].begin(),
            leftPrims[i].end()
        );
        rightPrimitives.insert(
            rightPrimitives.end(),
            rightPrims[i].begin(),
            rightPrims[i].end()
        );
    }
}
