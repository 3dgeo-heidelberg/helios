#include <FastSAHKDTreeFactory.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
FastSAHKDTreeFactory::FastSAHKDTreeFactory(size_t const lossNodes,
                                           double const ci,
                                           double const cl,
                                           double const co)
  : SAHKDTreeFactory(lossNodes, ci, cl, co)
{
}

// ***  CLONE  *** //
// *************** //
KDTreeFactory*
FastSAHKDTreeFactory::clone() const
{
  FastSAHKDTreeFactory* kdtf = new FastSAHKDTreeFactory(lossNodes, ci, cl, co);
  _clone(kdtf);
  return kdtf;
}

void
FastSAHKDTreeFactory::_clone(KDTreeFactory* kdtf) const
{
  SAHKDTreeFactory::_clone(kdtf);
}

// ***  SAH UTILS  *** //
// ******************* //
double
FastSAHKDTreeFactory::findSplitPositionBySAH(
  KDTreeNode* node,
  vector<Primitive*>& primitives) const
{
  return findSplitPositionByFastSAHRecipe(
    node,
    primitives,
    [&](vector<Primitive*>& primitives,
        int const splitAxis,
        double const minp,
        double const deltap,
        size_t const lossNodes,
        size_t const lossCases,
        vector<size_t>& cForward,
        vector<size_t>& cBackward) -> void {
      // Count min and max vertices
      vector<size_t> minCount(lossNodes, 0);
      vector<size_t> maxCount(lossNodes, 0);
      for (Primitive* q : primitives) {
        double const minq = q->getAABB()->getMin()[splitAxis];
        double const maxq = q->getAABB()->getMax()[splitAxis];
        ++minCount[std::min<size_t>(
          (size_t)((minq - minp) / deltap * lossNodes), lossNodes - 1)];
        ++maxCount[std::min<size_t>(
          (size_t)((maxq - minp) / deltap * lossNodes), lossNodes - 1)];
      }

      // Accumulate counts
      for (size_t i = 0; i < lossNodes; ++i) {
        cForward[i + 1] = cForward[i] + minCount[i];
      }
      for (size_t i = lossNodes; i > 0; --i) {
        cBackward[i - 1] = cBackward[i] + maxCount[i - 1];
      }
    });
}

double
FastSAHKDTreeFactory::findSplitPositionByFastSAHRecipe(
  KDTreeNode* node,
  vector<Primitive*>& primitives,
  std::function<void(vector<Primitive*>& primitives,
                     int const splitAxis,
                     double const minp,
                     double const deltap,
                     size_t const lossNodes,
                     size_t const lossCases,
                     vector<size_t>& cForward,
                     vector<size_t>& cBackward)> f_recount) const
{
  /*
   * Code below is commented because it might be necessary in the future.
   * If problems are found with the fast SAH implementation, notice that it
   *  is a well known issue that the min-max method might cause problems at
   *  nodes where number of primitives is smaller than the number of bins.
   *  In such cases, it is recommended to use the full SAH computation
   *  instead of the min-max approximation.
   *
   * If performance is not good enough, instead of using lossNodes for the
   *  full SAH and the fast SAH, implement a numBins attribute for the
   *  fast SAH. Then, use lossNodes just for full SAH. Good initial values
   *  for the average case would be 21 lossNodes for full SAH and 32 numBins
   *  for the fast SAH
   */
  // If there are not enough primitives, use a more accurate loss computation
  /*if(primitives.size() <= numBins)  // nBins = lossNodes
      return SAHKDTreeFactory::findSplitPositionBySAH(node, primitives);*/

  // Forward and backward count w.r.t. min and max vertices respectively
  double const minp = node->bound.getMin()[node->splitAxis];
  double const maxp = node->bound.getMax()[node->splitAxis];
  double const deltap = maxp - minp;
  size_t const lossCases = lossNodes + 1;
  vector<size_t> cForward(lossCases, 0);
  vector<size_t> cBackward(lossCases, 0);
  f_recount(primitives,
            node->splitAxis,
            minp,
            deltap,
            lossNodes,
            lossCases,
            cForward,
            cBackward);

  // Approximated discrete search of optimal splitting plane
  double loss = (double)cBackward[0], newLoss;
  node->splitPos = minp;
  double const _lossNodes = (double)lossNodes;
  for (size_t i = 1; i <= lossNodes; ++i) {
    double const r = ((double)i) / _lossNodes;
    newLoss = r * ((double)cForward[i]) + (1.0 - r) * ((double)cBackward[i]);
    if (newLoss < loss) {
      loss = newLoss;
      node->splitPos = (i < lossNodes) ? minp + r * deltap : maxp;
    }
  }

  // Store loss if requested
  return loss;
}
