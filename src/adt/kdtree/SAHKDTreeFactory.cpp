#include <SAHKDTreeFactory.h>

#include <KDTreePrimitiveComparator.h>
#include <fluxionum/UnivariateNewtonRaphsonMinimizer.h>

using fluxionum::UnivariateNewtonRaphsonMinimizer;

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

    // Obtain the object median
    size_t const m = primitives.size();
    std::sort(
        primitives.begin(),
        primitives.end(),
        KDTreePrimitiveComparator(node->splitAxis)
    );
    Primitive * medianObject = primitives[m/2]; // The median object
    double const objectMedian = medianObject->getCentroid()[node->splitAxis];

    // TODO Rethink : This is the Simple KDTree implementation. ...
    // ... It must be substituted by the SAH implementation
    node->splitPos = medianObject->getCentroid()[node->splitAxis];
    //return; // TODO Remove

    // TODO Rethink : Binning based search can be improved with spatial median
    // Discrete search of optimal splitting plane according to loss function
    double const a = node->bound.getMin()[node->splitAxis];
    double const b = node->bound.getMax()[node->splitAxis];
    double const length = b-a;
    double const mu = (b+a)/2.0;
    double const me = medianObject->getCentroid()[node->splitAxis];
    size_t const n = 1001; // Number of desired sampling/binning nodes
    double const n_1 = n-1;
    double const delta = (mu>me) ? mu-me : me-mu;
    double const start = (mu>me) ? me : mu;
    double const step = delta/n_1;
    double loss = std::numeric_limits<double>::max();
    loss = splitLoss( // TODO Remove (used to know loss of median in debug)
        primitives,
        node->splitAxis,
        node->splitPos,
        node->surfaceArea,
        (node->splitPos-a) / (b-a)
    );
    for(size_t i = 0 ; i < n ; ++i){
        double const phi = start + ((double)i)*step;
        double const newLoss = splitLoss(
            primitives,
            node->splitAxis,
            phi,
            node->surfaceArea,
            (phi-a) / length
        );
        if(newLoss < loss){
            loss = newLoss;
            node->splitPos = phi;
        }
    }
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
                kdtNode->surfaceArea * kdtNode->primitives.size();
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
    vector<Primitive *> const &leftPrimitives,
    vector<Primitive *> const &rightPrimitives
) {
    size_t const primsSize = primitives.size();
    // Compute parent heuristic (INIT ILOT)
    if(parent==nullptr){
        double hi, hl, ho, ht;
        toILOTCache(0.0, 0.0, 0.0, 0.0); // Initialize cache to 0
        heuristicILOT(
            hi, hl, ho, ht,
            node->surfaceArea,  0.0, node->surfaceArea,
            primitives
        );
        toILOTCache(hi, hl, ho, ht); // Set cache to C_T at t0
    }

    // Do partitioning
    if(
        leftPrimitives.size() != primsSize &&
        rightPrimitives.size() != primsSize
    ){ // If there are primitives on both partitions, binary split the node
        if(!leftPrimitives.empty()){
            node->left = buildRecursive(
                node,
                true,
                leftPrimitives,
                depth + 1
            );
        }
        if(!rightPrimitives.empty()){
            node->right = buildRecursive(
                node,
                false,
                rightPrimitives,
                depth + 1
            );
        }
    }
    else {
        // Otherwise, make this node a leaf:
        node->splitAxis = -1;
        node->primitives = primitives;
    }
}

// ***  SAH UTILS  *** //
// ******************* //
double SAHKDTreeFactory::splitLoss(
    vector<Primitive *> const &primitives,
    int const splitAxis,
    double const splitPos,
    double const surfaceArea,
    double const b
) const {
    // Split in left and right primitives
    vector<Primitive *> lps(0);
    vector<Primitive *> rps(0);
    for(Primitive *primitive : primitives){
        AABB const *primBox = primitive->getAABB();
        if(primBox->getMin()[splitAxis] <= splitPos) lps.push_back(primitive);
        if(primBox->getMax()[splitAxis] > splitPos) rps.push_back(primitive);
    }
    size_t const lpsSize = lps.size();
    size_t const rpsSize = rps.size();

    // Compute surface ares for left and right sides
    double const sal = surfaceArea * b;
    double const sar = surfaceArea * (1.0-b);

    // Compute and return loss function
    /*std::cout   << "lpsSize=" << lpsSize << ", "
                << "sal=" << sal << ",    "
                << "rpsSize=" << rpsSize << ", "
                << "sar=" << sar
                << std::endl;*/ // TODO Remove
    return sal*((double)lpsSize) + sar*((double)rpsSize);
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
    size_t const No = primitives.size();

    // Compute updates
    hi = this->ci * (this->cacheI+surfaceAreaInterior);
    hl = this->cl * (this->cacheL+surfaceAreaLeaf);
    ho = this->co * (this->cacheO+surfaceAreaLeaf*No);
    ht = (hi + hl + ho) / surfaceAreaRoot;
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
    ht = (this->ci*hi + this->cl*hl + this->co*ho) / saRoot;
    return ht;
}
