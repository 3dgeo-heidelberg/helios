#include <AxisSAHKDTreeGeometricStrategy.h>

// ***  CLONE  *** //
// *************** //
SimpleKDTreeGeometricStrategy * AxisSAHKDTreeGeometricStrategy::clone(
    SimpleKDTreeFactory *kdtf
) const{
    return new AxisSAHKDTreeGeometricStrategy(*((AxisSAHKDTreeFactory *)kdtf));
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
void AxisSAHKDTreeGeometricStrategy::GEOM_defineSplit(
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
