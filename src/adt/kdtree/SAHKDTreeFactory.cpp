#include <SAHKDTreeFactory.h>


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SAHKDTreeFactory::SAHKDTreeFactory (
    size_t const lossNodes,
    double const ci,
    double const cl,
    double const co
) :
    SimpleKDTreeFactory(),
    lossNodes(21),
    ci(ci),
    cl(cl),
    co(co)
{
    /*
     * See SimpleKDTreeFactory constructor implementation to understand why
     *  it is safe to call virtual function here.
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
    _lockILOT = [] () -> void {return;};
    _unlockILOT = [] () -> void {return;};
}
// ***  BUILDING METHODS  *** //
// ************************** //
void SAHKDTreeFactory::defineSplit(
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> &primitives,
    int const depth
) const {
    // Find split axis
    node->splitAxis = depth % 3;
    findSplitPositionBySAH(node, primitives);
};

void SAHKDTreeFactory::computeKDTreeStats(KDTreeNodeRoot *root) const{
    // Compute basic KDTree stats
    SimpleKDTreeFactory::computeKDTreeStats(root);

    // Compute tree cost
    double const saRoot = root->surfaceArea;
    double interiorAreaSum = 0.0;
    double leafAreaSum = 0.0;
    double leafObjectAreaSum = 0.0;
    BinaryTreeDepthIterator<KDTreeNode> btdi(root, 0);
    while(btdi.hasNext()){
        IterableTreeNode<IBinaryTreeNode> node = btdi.next();
        KDTreeNode const * kdtNode = static_cast<KDTreeNode *>(node.getNode());
        if(kdtNode->isLeafNode()){
            leafAreaSum += kdtNode->surfaceArea;
            leafObjectAreaSum +=
                kdtNode->surfaceArea * ((double)kdtNode->primitives->size());
        }
        else{
            interiorAreaSum += kdtNode->surfaceArea;
        }
    }
    root->stats_totalCost =
        (ci*interiorAreaSum + cl*leafAreaSum + co*leafObjectAreaSum) / saRoot;
}

void SAHKDTreeFactory::buildChildrenNodes(
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> const &primitives,
    int const depth,
    int const index,
    vector<Primitive *> &leftPrimitives,
    vector<Primitive *> &rightPrimitives
) {
    // Call recipe to build children nodes
    buildChildrenNodesRecipe(
        node,
        parent,
        primitives,
        depth,
        index,
        leftPrimitives,
        rightPrimitives,
        [&] (
            KDTreeNode *node,
            int const depth,
            int const index,
            vector<Primitive *> &leftPrimitives,
            vector<Primitive *> &rightPrimitives
        ) -> void {
            if(!leftPrimitives.empty()){ // Build left child
                setChild(node->left, _buildRecursive(
                    node, true, leftPrimitives, depth + 1, 2*index
                ));
            }
            if(!rightPrimitives.empty()){ // Build right child
                setChild(node->right, _buildRecursive(
                    node, false, rightPrimitives, depth + 1, 2*index + 1
                    ));
            }
        }
    );
}

void SAHKDTreeFactory::buildChildrenNodesRecipe(
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> const &primitives,
    int const depth,
    int const index,
    vector<Primitive *> &leftPrimitives,
    vector<Primitive *> &rightPrimitives,
    std::function<void(
        KDTreeNode *node,
        int const depth,
        int const index,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives
    )>f_buildChildrenNodes
){
    if(parent==nullptr){ // Compute parent heuristic (INIT ILOT)
        initILOT(node, primitives); // Lock not need, only sequential
    }

    bool split = \
        checkNodeMustSplit(primitives, leftPrimitives, rightPrimitives);
    if(split){ // If split is possible because there are enough primitives
        // Compute node as internal
        double hi, hl, ho, ht;
        _lockILOT(); // Lock ILOT cache
        internalizeILOT(
            hi, hl, ho, ht,
            node, primitives, leftPrimitives, rightPrimitives
        );

        // Do partitioning if new cost is smaller than previous one
        if(ht < getCacheT()){
            toILOTCache(hi, hl, ho, ht);  // Update heuristic ILOT cache
            _unlockILOT(); // Unlock ILOT cache
            f_buildChildrenNodes(
                node,
                depth,
                index,
                leftPrimitives,
                rightPrimitives
            );
        }
        else { // But if cost is not smaller than previous one
            _unlockILOT(); // Unlock ILOT cache if finally not splitted
            split = false; // Flag as not splitted
        }
    }
    if(!split){ // If no split, then make this node a leaf
        makeLeaf(node, primitives);
    }
}

// ***  BUILDING UTILS  *** //
// ************************ //
bool SAHKDTreeFactory::checkNodeMustSplit(
    vector<Primitive *> const &primitives,
    vector<Primitive *> const &leftPrimitives,
    vector<Primitive *> const &rightPrimitives
) const {
    return primitives.size() >= minSplitPrimitives;
}

// ***  SAH UTILS  *** //
// ******************* //
double SAHKDTreeFactory::splitLoss(
    vector<Primitive *> const &primitives,
    int const splitAxis,
    double const splitPos,
    double const r
) const {
    // Split in left and right primitives
    size_t lps = 0, rps = 0;
    for(Primitive *primitive : primitives){
        AABB const *primBox = primitive->getAABB();
        if(primBox->getMin()[splitAxis] <= splitPos) ++lps;
        if(primBox->getMax()[splitAxis] > splitPos) ++rps;
    }

    // Compute and return loss function
    return r*((double)lps) + (1.0-r)*((double)rps);
}

void SAHKDTreeFactory::computeBestSplit(
    vector<Primitive *> &primitives,
    size_t const lossNodes,
    double const start,
    double const step,
    int const splitAxis,
    double const minBound,
    double const boundLength,
    double &loss,
    double &splitPos
) const {
    for(size_t i = 0 ; i < lossNodes ; ++i){
        double const phi = start + ((double)i)*step;
        double const newLoss = splitLoss(
            primitives,
            splitAxis,
            phi,
            (phi-minBound) / boundLength
        );
        if(newLoss < loss){
            loss = newLoss;
            splitPos = phi;
        }
    }
}

double SAHKDTreeFactory::findSplitPositionBySAH(
    KDTreeNode *node,
    vector<Primitive *> &primitives
) const{
    // Call recipe to find split position by SAH
    return findSplitPositionBySAHRecipe(
        node,
        primitives,
        [&] (
            vector<Primitive *>::iterator begin,
            vector<Primitive *>::iterator end,
            KDTreePrimitiveComparator comparator
        ) -> void {
            std::sort(begin, end, comparator);
        },
        [&] (
            vector<Primitive *> &primitives,
            size_t const lossNodes,
            double const start,
            double const step,
            int const splitAxis,
            double const minBound,
            double const boundLength,
            double &loss,
            double &splitPos
        ) -> void {
            computeBestSplit(
                primitives,
                lossNodes,
                start,
                step,
                splitAxis,
                minBound,
                boundLength,
                loss,
                node->splitPos
            );
        }
    );
}

double SAHKDTreeFactory::findSplitPositionBySAHRecipe(
    KDTreeNode *node,
    vector<Primitive *> &primitives,
    std::function<void(
        vector<Primitive *>::iterator begin,
        vector<Primitive *>::iterator end,
        KDTreePrimitiveComparator comparator
    )> f_sortPrimitives,
    std::function<void(
        vector<Primitive *> &primitives,
        size_t const lossNodes,
        double const start,
        double const step,
        int const splitAxis,
        double const minBound,
        double const boundLength,
        double &loss,
        double &splitPos
    )> f_computeLossNodes
) const {
    // Obtain the object median
    f_sortPrimitives(
        primitives.begin(),
        primitives.end(),
        KDTreePrimitiveComparator(node->splitAxis)
    );

    // Discrete search of optimal splitting plane according to loss function
    double const a = node->bound.getMin()[node->splitAxis];
    double const b = node->bound.getMax()[node->splitAxis];
    double const length = b-a;
    double const mu = (b+a)/2.0;
    double me = primitives[primitives.size()/2]->getCentroid()[node->splitAxis];
    if(me < a) me = a;
    if(me > b) me = b;
    double const start = (mu>me) ? me : mu;
    double const step = (mu>me) ? (mu-me)/((double)(lossNodes-1)) :
                        (me-mu)/((double)(lossNodes-1));
    double loss = std::numeric_limits<double>::max();
    f_computeLossNodes(
        primitives,
        lossNodes,
        start,
        step,
        node->splitAxis,
        a,
        length,
        loss,
        node->splitPos
    );
    return loss;
}

double SAHKDTreeFactory::heuristicILOT(
    double &hi,
    double &hl,
    double &ho,
    double &ht,
    double const surfaceAreaRoot,
    double const surfaceAreaInterior,
    double const surfaceAreaLeaf,
    vector<Primitive *> const &primitives
) const {
    // Extract constants of interest
    double const No = (double) primitives.size();

    // Compute updates
    double I, L, O;
    fromILOCache(I, L, O);
    hi = I + this->ci*surfaceAreaInterior;
    hl = L + this->cl*surfaceAreaLeaf;
    ho = O + this->co*surfaceAreaLeaf*No;
    ht = (hi + hl + ho) / surfaceAreaRoot;

    // Return cost
    return ht;
}
double SAHKDTreeFactory::cumulativeILOT(
    double &hi,
    double &hl,
    double &ho,
    double &ht,
    double const _hi,
    double const _hl,
    double const _ho,
    double const saRoot
) const {
    hi += _hi;
    hl += _hl;
    ho += _ho;
    ht = (hi + hl + ho) / saRoot;
    return ht;
}

void SAHKDTreeFactory::internalizeILOT(
    double &hi,
    double &hl,
    double &ho,
    double &ht,
    KDTreeNode *node,
    vector<Primitive *> const &primitives,
    vector<Primitive *> const &leftPrimitives,
    vector<Primitive *> const &rightPrimitives
){
    heuristicILOT( // Subtract old leaf cost and add new interior cost
        hi, hl, ho, ht,
        cacheRoot->surfaceArea,
        node->surfaceArea,
        -node->surfaceArea,
        primitives
    );
    double const a = node->bound.getMin()[node->splitAxis];
    double const b = node->bound.getMax()[node->splitAxis];
    double const p = node->splitPos;
    double const r = (p-a)/(b-a);
    double const leftSurfaceArea = r * node->surfaceArea;
    double const rightSurfaceArea = (1.0-r) * node->surfaceArea;
    cumulativeILOT( // Cumulative of leaf cost for left and right splits
        hi, hl, ho, ht,
        0.0,
        this->cl * (leftSurfaceArea + rightSurfaceArea),
        this->co * (
            leftSurfaceArea * ((double)leftPrimitives.size()) +
            rightSurfaceArea * ((double)rightPrimitives.size())
        ),
        cacheRoot->surfaceArea
    );
}

// ***  CACHE UTILS  *** //
// ********************* //
void SAHKDTreeFactory::initILOT(
    KDTreeNode *root,
    vector<Primitive *> const &primitives
){
    double hi, hl, ho, ht;
    setCacheRoot(root); // Cache root node to compute future children's ILOT
    toILOTCache(0.0, 0.0, 0.0, 0.0); // Initialize cache to 0
    heuristicILOT(
        hi, hl, ho, ht,         // Output variables
        root->surfaceArea,      // Surface area root
        0.0,                    // Surface area interior
        root->surfaceArea,      // Surface area leaf
        primitives              // Contained primitives
    );
    toILOTCache(hi, hl, ho, ht); // Set cache to C_T at t0
}
