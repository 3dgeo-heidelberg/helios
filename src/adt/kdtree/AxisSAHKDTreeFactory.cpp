#include <helios/adt/kdtree/AxisSAHKDTreeFactory.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
AxisSAHKDTreeFactory::AxisSAHKDTreeFactory(size_t const lossNodes,
                                           double const ci,
                                           double const cl,
                                           double const co)
  : SAHKDTreeFactory(lossNodes, ci, cl, co)
{
}

// ***  CLONE  *** //
// *************** //
KDTreeFactory*
AxisSAHKDTreeFactory::clone() const
{
  AxisSAHKDTreeFactory* kdtf = new AxisSAHKDTreeFactory(lossNodes, ci, cl, co);
  _clone(kdtf);
  return kdtf;
}

void
AxisSAHKDTreeFactory::_clone(KDTreeFactory* kdtf) const
{
  SAHKDTreeFactory::_clone(kdtf);
}

// ***  BUILDING METHODS  *** //
// ************************** //
void
AxisSAHKDTreeFactory::defineSplit(KDTreeNode* node,
                                  KDTreeNode* parent,
                                  std::vector<Primitive*>& primitives,
                                  int const depth) const
{
  // Initial as best
  node->splitAxis = 0;
  int bestSplitAxis = 0;
  double bestLoss = findSplitPositionBySAH(node, primitives);
  double bestSplitPos = node->splitPos;

  // Iterative search
  for (int newSplitAxis = 1; newSplitAxis < 3; ++newSplitAxis) {
    node->splitAxis = newSplitAxis;
    double const newLoss = findSplitPositionBySAH(node, primitives);
    if (newLoss < bestLoss) {
      bestLoss = newLoss;
      bestSplitAxis = node->splitAxis;
      bestSplitPos = node->splitPos;
    }
  }

  // Assign best
  node->splitAxis = bestSplitAxis;
  node->splitPos = bestSplitPos;
}
