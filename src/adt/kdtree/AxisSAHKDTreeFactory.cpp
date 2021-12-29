#include <AxisSAHKDTreeFactory.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
AxisSAHKDTreeFactory::AxisSAHKDTreeFactory (
    size_t const lossNodes,
    double const ci,
    double const cl,
    double const co
) : SAHKDTreeFactory(lossNodes, ci, cl, co)
{}

// ***  BUILDING METHODS  *** //
// ************************** //
void AxisSAHKDTreeFactory::defineSplit(
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> &primitives,
    int const depth
) const {
    // Initial as best
    node->splitAxis = 0;
    int bestSplitAxis = 0;
    double bestLoss = findSplitPositionBySAH(node, primitives);
    double bestSplitPos = node->splitPos;

    // Iterative search
    for(int newSplitAxis = 1 ; newSplitAxis < 3 ; ++newSplitAxis){
        node->splitAxis = newSplitAxis;
        double const newLoss = findSplitPositionBySAH(node, primitives);
        if(newLoss < bestLoss){
            bestLoss = newLoss;
            bestSplitAxis = node->splitAxis;
            bestSplitPos = node->splitPos;
        }
    }

    // Assign best
    node->splitAxis = bestSplitAxis;
    node->splitPos = bestSplitPos;
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
void AxisSAHKDTreeFactory::GEOM_defineSplit(
    KDTreeNode *node,
    KDTreeNode *parent,
    vector<Primitive *> &primitives,
    int const depth,
    int const assignedThreads
) const{
    // Initial as best
    node->splitAxis = 0;
    int bestSplitAxis = 0;
    double bestLoss = GEOM_findSplitPositionBySAH(
        node, primitives, assignedThreads
    );
    double bestSplitPos = node->splitPos;

    // Iterative search
    for(int newSplitAxis = 1 ; newSplitAxis < 3 ; ++newSplitAxis){
        node->splitAxis = newSplitAxis;
        double const newLoss = GEOM_findSplitPositionBySAH(
            node, primitives, assignedThreads
        );
        if(newLoss < bestLoss){
            bestLoss = newLoss;
            bestSplitAxis = node->splitAxis;
            bestSplitPos = node->splitPos;
        }
    }

    // Assign best
    node->splitAxis = bestSplitAxis;
    node->splitPos = bestSplitPos;
}
