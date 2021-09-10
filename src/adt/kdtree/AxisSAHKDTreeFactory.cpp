#include <AxisSAHKDTreeFactory.h>

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
