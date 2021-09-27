#include <AxisSAHKDTreeFactory.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
/**
 * @brief Axis surface area heuristic KDTree factory default constructor
 * @see SAHKDTreeFactory::SAHKDTreeFactory
 */
AxisSAHKDTreeFactory::AxisSAHKDTreeFactory (
    size_t const lossNodes,
    double const ci,
    double const cl,
    double const co
) : SAHKDTreeFactory(lossNodes, ci, cl, co)
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
            int const depth
        ) -> KDTreeNode * {
            return this->buildRecursive(parent, left, primitives, depth);
        }
    ;
}

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
