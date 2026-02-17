#include <helios/adt/kdtree/FastSAHKDTreeGeometricStrategy.h>
#include <helios/adt/kdtree/FastSAHKDTreeRecountSubTask.h>

// ***  CLONE  *** //
// *************** //
SimpleKDTreeGeometricStrategy*
FastSAHKDTreeGeometricStrategy::clone(SimpleKDTreeFactory* kdtf) const
{
  return new FastSAHKDTreeGeometricStrategy(*((FastSAHKDTreeFactory*)kdtf));
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
double
FastSAHKDTreeGeometricStrategy::GEOM_findSplitPositionBySAH(
  KDTreeNode* node,
  std::vector<Primitive*>& primitives,
  int assignedThreads) const
{
  // Handle cases where sequential execution is preferred
  if (assignedThreads == 1 || ((int)primitives.size()) < 2 * assignedThreads) {
    return fsahkdtf.findSplitPositionBySAH(node, primitives);
  }
  // Handle multi-thread cases
  return fsahkdtf.findSplitPositionByFastSAHRecipe(
    node,
    primitives,
    [&](std::vector<Primitive*>& primitives,
        int const splitAxis,
        double const minp,
        double const deltap,
        std::size_t const lossNodes,
        std::size_t const lossCases,
        std::vector<std::size_t>& cForward,
        std::vector<std::size_t>& cBackward)
      -> void { // Count forward and backward
      // Distribute workload
      std::size_t const numPrimitives = primitives.size();
      /*
       * Using assignedThreads = min(assignedThreads, numPrimitives)
       *  might degrade performance because the overhead of handling
       *  thread contexts could overcome the improvement from
       *  parallelization.
       * If this is detected, switching strategy to something like
       *  assignedThreads = min(assignedThreads, numPrimitives/k),
       *  for k > 1, might alleviate the problem with an adequate k.
       * However, this is expected to be unlikely because geometry-level
       *  parallelization is applied on upper tree nodes, where the
       *  number of primitives stills high.
       */
      if (assignedThreads > (int)numPrimitives)
        assignedThreads = (int)numPrimitives;
      std::size_t const chunkSize =
        numPrimitives / ((std::size_t)assignedThreads);
      std::vector<std::vector<std::size_t>> lForward(
        assignedThreads, std::vector<std::size_t>(lossCases, 0));
      std::vector<std::vector<std::size_t>> lBackward = lForward;
      int const extraThreads = assignedThreads - 1;
      std::shared_ptr<SharedTaskSequencer> stSequencer =
        std::make_shared<SharedTaskSequencer>(extraThreads);
      // Parallel compute auxiliar threads
      for (int i = 0; i < extraThreads; ++i) {
        stSequencer->start(std::make_shared<FastSAHKDTreeRecountSubTask>(
          stSequencer,
          splitAxis,
          minp,
          deltap,
          primitives.begin() + i * chunkSize,
          primitives.begin() + (i + 1) * chunkSize,
          lossNodes,
          lossCases,
          lForward[i],
          lBackward[i]));
      }
      // Main thread computation
      FastSAHKDTreeRecountSubTask(nullptr,
                                  splitAxis,
                                  minp,
                                  deltap,
                                  primitives.begin() + extraThreads * chunkSize,
                                  primitives.end(),
                                  lossNodes,
                                  lossCases,
                                  lForward.back(),
                                  lBackward.back())();

      // Wait until workload has been consumed
      stSequencer->joinAll();

      // Reduce local counts to counts
      for (std::size_t i = 0; i < lossCases; ++i) {
        for (std::vector<std::size_t>& lf : lForward) {
          cForward[i] += lf[i];
        }
        for (std::vector<std::size_t>& lb : lBackward) {
          cBackward[i] += lb[i];
        }
      }
    });
}
