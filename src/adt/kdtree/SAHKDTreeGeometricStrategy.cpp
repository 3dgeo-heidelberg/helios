#include <helios/adt/kdtree/SAHKDTreeComputeLossNodesSubTask.h>
#include <helios/adt/kdtree/SAHKDTreeGeometricStrategy.h>
#include <helios/adt/kdtree/SimpleKDTreeBuildChildrenNodesSubTask.h>
#include <helios/hpc/SM_ParallelMergeSort.h>

using helios::hpc::SM_ParallelMergeSort;

// ***  CLONE  *** //
// *************** //
SimpleKDTreeGeometricStrategy*
SAHKDTreeGeometricStrategy::clone(SimpleKDTreeFactory* kdtf) const
{
  return new SAHKDTreeGeometricStrategy(*((SAHKDTreeFactory*)kdtf));
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
void
SAHKDTreeGeometricStrategy::GEOM_defineSplit(
  KDTreeNode* node,
  KDTreeNode* parent,
  std::vector<Primitive*>& primitives,
  int const depth,
  int const assignedThreads) const
{
  // Find split axis
  node->splitAxis = depth % 3;
  GEOM_findSplitPositionBySAH(node, primitives, assignedThreads);
}

double
SAHKDTreeGeometricStrategy::GEOM_findSplitPositionBySAH(
  KDTreeNode* node,
  std::vector<Primitive*>& primitives,
  int assignedThreads) const
{
  if (assignedThreads < 2) { // Sequential search
    return sahkdtf.findSplitPositionBySAH(node, primitives);
  } else { // Parallel search
    return sahkdtf.findSplitPositionBySAHRecipe(
      node,
      primitives,
      [&](std::vector<Primitive*>::iterator begin,
          std::vector<Primitive*>::iterator end,
          KDTreePrimitiveComparator comparator) -> void {
        SM_ParallelMergeSort<std::vector<Primitive*>::iterator,
                             KDTreePrimitiveComparator>
          sorter(assignedThreads, assignedThreads * 2);
        sorter.sort(primitives.begin(),
                    primitives.end(),
                    KDTreePrimitiveComparator(node->splitAxis));
      },
      [&](std::vector<Primitive*>& primitives,
          std::size_t const lossNodes,
          double const start,
          double const step,
          int const splitAxis,
          double const minBound,
          double const boundLength,
          double& loss,
          double& splitPos) -> void {
        // Distribute workload
        /**
         * Using assignedThreads = min(assignedThreads, lossNodes) is
         *  expected to work properly because each node requires
         *  iterating over the entire set of primitives, which during
         *  upper tree levels (those handled by
         *      geometry-level parallelization) is expected to be high.
         * However, if performance problems are diagnosed when using
         *  lossNodes >= assignedThreads or even
         *  lossNodes+x >= assignedThreads for small values of x,
         *  then switching strategy to something like
         *  assignedThreads = min(assignedThreads, lossNodes/k),
         *  for k > 1, might alleviate the problem with an adequate k.
         */
        if (assignedThreads > (int)lossNodes)
          assignedThreads = (int)lossNodes;
        std::size_t const chunkSize =
          lossNodes / ((std::size_t)assignedThreads);
        int const extraThreads = assignedThreads - 1;
        std::shared_ptr<SharedTaskSequencer> stSequencer =
          std::make_shared<SharedTaskSequencer>(extraThreads);
        std::vector<double> partialLoss(extraThreads, loss);
        std::vector<double> partialSplitPos(extraThreads, 0);
        for (int i = 0; i < extraThreads; ++i) {
          stSequencer->start(std::make_shared<SAHKDTreeComputeLossNodesSubTask>(
            stSequencer,
            primitives,
            start,
            step,
            node->splitAxis,
            minBound,
            boundLength,
            i * chunkSize,
            (i + 1) * chunkSize,
            partialLoss[i],
            partialSplitPos[i],
            [&](std::vector<Primitive*> const& primitives,
                int const splitAxis,
                double const splitPos,
                double const r) -> double {
              return sahkdtf.splitLoss(primitives, splitAxis, splitPos, r);
            }));
        }
        SAHKDTreeComputeLossNodesSubTask(
          nullptr,
          primitives,
          start,
          step,
          node->splitAxis,
          minBound,
          boundLength,
          extraThreads * chunkSize,
          lossNodes,
          partialLoss[extraThreads],
          partialSplitPos[extraThreads],
          [&](std::vector<Primitive*> const& primitives,
              int const splitAxis,
              double const splitPos,
              double const r) -> double {
            return sahkdtf.splitLoss(primitives, splitAxis, splitPos, r);
          });

        // Wait until workload has been consumed
        stSequencer->joinAll();

        // Reduce to best split
        loss = partialLoss[0];
        node->splitPos = partialSplitPos[0];
        for (int i = 1; i < extraThreads; ++i) {
          if (partialLoss[i] < loss) {
            loss = partialLoss[i];
            node->splitPos = partialSplitPos[i];
          }
        }
      });
  }
}

void
SAHKDTreeGeometricStrategy::GEOM_buildChildrenNodes(
  KDTreeNode* node,
  KDTreeNode* parent,
  std::vector<Primitive*> const& primitives,
  int const depth,
  int const index,
  std::vector<Primitive*>& leftPrimitives,
  std::vector<Primitive*>& rightPrimitives,
  std::shared_ptr<SharedTaskSequencer> masters)
{
  // Call recipe to build children nodes
  sahkdtf.buildChildrenNodesRecipe(
    node,
    parent,
    primitives,
    depth,
    index,
    leftPrimitives,
    rightPrimitives,
    [&](KDTreeNode* node,
        int const depth,
        int const index,
        std::vector<Primitive*>& leftPrimitives,
        std::vector<Primitive*>& rightPrimitives) -> void {
      std::shared_ptr<SimpleKDTreeBuildChildrenNodesSubTask> task = nullptr;
      bool buildRightNode = !rightPrimitives.empty();
      if (buildRightNode) {
        task = std::make_shared<SimpleKDTreeBuildChildrenNodesSubTask>(
          masters,
          node,
          rightPrimitives,
          depth,
          index,
          [&](LightKDTreeNode*& child, KDTreeNode* node) -> void {
            sahkdtf.setChild(child, node);
          },
          sahkdtf._buildRecursive);
        masters->start(task);
      }
      if (!leftPrimitives.empty()) { // Build left child
        sahkdtf.setChild(node->left,
                         sahkdtf._buildRecursive(
                           node, true, leftPrimitives, depth + 1, 2 * index));
      }
      if (buildRightNode) { // Wait until right child node has been built
        task->getThread()->join();
      }
    });
}
