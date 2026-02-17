#include <helios/adt/kdtree/KDTreePrimitiveComparator.h>
#include <helios/adt/kdtree/SimpleKDTreeBuildChildrenNodesSubTask.h>
#include <helios/adt/kdtree/SimpleKDTreeComputeRootNodeBoundariesSubTask.h>
#include <helios/adt/kdtree/SimpleKDTreeGeometricStrategy.h>
#include <helios/adt/kdtree/SimpleKDTreePopulateSplitsSubTask.h>
#include <helios/hpc/SM_ParallelMergeSort.h>
#include <helios/surfaceinspector/maths/Vector.hpp>

using helios::hpc::SM_ParallelMergeSort;
using SurfaceInspector::maths::Vector;

// ***  CLONE  *** //
// *************** //
SimpleKDTreeGeometricStrategy*
SimpleKDTreeGeometricStrategy::clone(SimpleKDTreeFactory* kdtf) const
{
  return new SimpleKDTreeGeometricStrategy(*kdtf);
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
void
SimpleKDTreeGeometricStrategy::GEOM_defineSplit(
  KDTreeNode* node,
  KDTreeNode* parent,
  std::vector<Primitive*>& primitives,
  int const depth,
  int const assignedThreads) const
{
  // Find split axis
  node->splitAxis = depth % 3;

  // Sort faces along split axis:
  // ATTENTION: Sorting must happen BEFORE splitPos is computed as the median
  // Sort primitives along split axis:
  SM_ParallelMergeSort<std::vector<Primitive*>::iterator,
                       KDTreePrimitiveComparator>
    sorter(assignedThreads, assignedThreads * 2);
  sorter.sort(primitives.begin(),
              primitives.end(),
              KDTreePrimitiveComparator(node->splitAxis));

  // Compute split position from centroid of median primitive
  auto p = next(primitives.begin(), primitives.size() / 2);
  node->splitPos = (*p)->getCentroid()[node->splitAxis];
}

void
SimpleKDTreeGeometricStrategy::GEOM_computeNodeBoundaries(
  KDTreeNode* node,
  KDTreeNode* parent,
  bool const left,
  std::vector<Primitive*> const& primitives,
  int assignedThreads)
{
  // Find surface area and minimum and maximum positions for root node
  if (parent == nullptr) {
    // Distribute workload
    std::size_t const numPrimitives = primitives.size();
    /*
     * Using assignedThreads = min(assignedThreads, numPrimitives)
     *  might degrade performance because the overhead of handling thread
     *  contexts could overcome the improvement from parallelization.
     * If this is detected, switching strategy to something like
     *  assignedThreads = min(assignedThreads, numPrimitives/k), for k > 1,
     *  might alleviate the problem with an adequate k.
     * However, this is expected to be unlikely because geometry-level
     *  parallelization is applied on upper tree nodes, where the number
     *  of primitives stills high.
     */
    if (assignedThreads > (int)numPrimitives)
      assignedThreads = (int)numPrimitives;
    std::size_t const chunkSize =
      numPrimitives / ((std::size_t)assignedThreads);
    int const extraThreads = assignedThreads - 1;
    std::shared_ptr<SharedTaskSequencer> stSequencer =
      std::make_shared<SharedTaskSequencer>(extraThreads);
    std::vector<double> Ax(assignedThreads, std::numeric_limits<double>::max());
    std::vector<double> Ay = Ax, Az = Ax;
    std::vector<double> Bx(assignedThreads,
                           std::numeric_limits<double>::lowest());
    std::vector<double> By = Bx, Bz = Bx;
    for (int i = 0; i < extraThreads; ++i) {
      stSequencer->start(
        std::make_shared<SimpleKDTreeComputeRootNodeBoundariesSubTask>(
          stSequencer,
          primitives,
          i * chunkSize,
          (i + 1) * chunkSize,
          Ax[i],
          Ay[i],
          Az[i],
          Bx[i],
          By[i],
          Bz[i],
          [&](Primitive* primitive,
              double& ax,
              double& ay,
              double& az,
              double& bx,
              double& by,
              double& bz) -> void {
            kdtf.onRootBoundariesDigestPrimitive(
              primitive, ax, ay, az, bx, by, bz);
          }));
    }
    SimpleKDTreeComputeRootNodeBoundariesSubTask(
      nullptr,
      primitives,
      extraThreads * chunkSize,
      numPrimitives,
      Ax[extraThreads],
      Ay[extraThreads],
      Az[extraThreads],
      Bx[extraThreads],
      By[extraThreads],
      Bz[extraThreads],
      [&](Primitive* primitive,
          double& ax,
          double& ay,
          double& az,
          double& bx,
          double& by,
          double& bz) -> void {
        kdtf.onRootBoundariesDigestPrimitive(primitive, ax, ay, az, bx, by, bz);
      })();

    // Wait until workload has been consumed
    stSequencer->joinAll();

    // Reduce (a,b)
    double const ax = Vector<double>::min(Ax);
    double const ay = Vector<double>::min(Ay);
    double const az = Vector<double>::min(Az);
    double const bx = Vector<double>::max(Bx);
    double const by = Vector<double>::max(By);
    double const bz = Vector<double>::max(Bz);

    // Compute surface area
    kdtf.onComputeNodeBoundariesCalcSAH(node, ax, ay, az, bx, by, bz);
  }
  // Find surface area and minimum and maximum positions for child node
  else {
    kdtf.computeMinMaxSAHForChild(node, parent, left, primitives);
  }
}

void
SimpleKDTreeGeometricStrategy::GEOM_populateSplits(
  std::vector<Primitive*> const& primitives,
  int const splitAxis,
  double const splitPos,
  std::vector<Primitive*>& leftPrimitives,
  std::vector<Primitive*>& rightPrimitives,
  int assignedThreads) const
{
  // Distribute workload
  std::size_t const numPrimitives = primitives.size();
  if (assignedThreads > (int)numPrimitives)
    assignedThreads = (int)numPrimitives;
  std::size_t const chunkSize = numPrimitives / ((std::size_t)assignedThreads);
  std::vector<std::vector<Primitive*>> leftPrims(assignedThreads);
  std::vector<std::vector<Primitive*>> rightPrims(assignedThreads);
  int const extraThreads = assignedThreads - 1;
  std::shared_ptr<SharedTaskSequencer> stSequencer =
    std::make_shared<SharedTaskSequencer>(extraThreads);
  for (int i = 0; i < extraThreads; ++i) {
    stSequencer->start(std::make_shared<SimpleKDTreePopulateSplitsSubTask>(
      stSequencer,
      primitives,
      splitAxis,
      splitPos,
      leftPrims[i],
      rightPrims[i],
      i * chunkSize,
      (i + 1) * chunkSize,
      [&](Primitive* p,
          int const splitAxis,
          double const splitPos,
          std::vector<Primitive*>& leftPrimitives,
          std::vector<Primitive*>& rightPrimitives) -> void {
        kdtf.onPopulateSplitsDigestPrimitive(
          p, splitAxis, splitPos, leftPrimitives, rightPrimitives);
      }));
  }
  SimpleKDTreePopulateSplitsSubTask(
    nullptr,
    primitives,
    splitAxis,
    splitPos,
    leftPrims[extraThreads],
    rightPrims[extraThreads],
    extraThreads * chunkSize,
    numPrimitives,
    [&](Primitive* p,
        int const splitAxis,
        double const splitPos,
        std::vector<Primitive*>& leftPrimitives,
        std::vector<Primitive*>& rightPrimitives) -> void {
      kdtf.onPopulateSplitsDigestPrimitive(
        p, splitAxis, splitPos, leftPrimitives, rightPrimitives);
    })();

  // Wait until workload has been consumed
  stSequencer->joinAll();

  // Reduce left and right primitives into single vector
  for (int i = 0; i < assignedThreads; i++) {
    leftPrimitives.insert(
      leftPrimitives.end(), leftPrims[i].begin(), leftPrims[i].end());
    rightPrimitives.insert(
      rightPrimitives.end(), rightPrims[i].begin(), rightPrims[i].end());
  }
}

void
SimpleKDTreeGeometricStrategy::GEOM_buildChildrenNodes(
  KDTreeNode* node,
  KDTreeNode* parent,
  std::vector<Primitive*> const& primitives,
  int const depth,
  int const index,
  std::vector<Primitive*>& leftPrimitives,
  std::vector<Primitive*>& rightPrimitives,
  std::shared_ptr<SharedTaskSequencer> masters)
{
  // If there are primitives on both partitions, binary split the node
  if (kdtf.checkNodeMustSplit(primitives, leftPrimitives, rightPrimitives)) {
    std::shared_ptr<SimpleKDTreeBuildChildrenNodesSubTask> task = nullptr;
    bool buildRightNode = !rightPrimitives.empty();
    if (buildRightNode) { // Delegate right child node to another thread
      task = std::make_shared<SimpleKDTreeBuildChildrenNodesSubTask>(
        masters,
        node,
        rightPrimitives,
        depth,
        index,
        [&](LightKDTreeNode*& child, KDTreeNode* node) -> void {
          kdtf.setChild(child, node);
        },
        kdtf._buildRecursive);
      masters->start(task);
    }
    if (!leftPrimitives.empty()) { // Compute left child node
      kdtf.setChild(
        node->left,
        kdtf._buildRecursive(node, true, leftPrimitives, depth + 1, 2 * index));
    }
    if (buildRightNode) { // Wait until right child node has been built
      task->getThread()->join();
    }
  } else { // Otherwise, make this node a leaf:
    kdtf.makeLeaf(node, primitives);
  }
}
