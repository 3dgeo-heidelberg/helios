#pragma once

#include <Primitive.h>
#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>

#include <vector>


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Shared sub-task to compute loss nodes when finding the split position
 *  during SAH KDTree building. It is meant to be used at geometry-level
 *  parallelization context and only for right child nodes.
 * @see SharedSubTask
 * @see SAHKDTreeFactory
 * @see SAHKDTreeFactory::findSplitPositionBySAH
 * @see MultiThreadKDTreeFactory
 */
class SAHKDTreeComputeLossNodesSubTask : public SharedSubTask
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Primitives being splitted to compute corresponding loss
   *  function
   */
  std::vector<Primitive*> const& primitives;
  /**
   * @brief Start split position
   */
  double const start;
  /**
   * @brief Step size between consecutive split positions
   */
  double const step;
  /**
   * @brief Split axis for current partition case
   */
  int const splitAxis;
  /**
   * @brief Min coordinate of boundary
   */
  double const minBound;
  /**
   * @brief Boundary length
   */
  double const boundLength;
  /**
   * @brief Start node (inclusive) for the iterative workload that must be
   *  computed by this sub-task
   */
  size_t const startNode;
  /**
   * @brief End node (exclusive) for the iterative workload that must be
   *  computed by this sub-task
   */
  size_t const endNode;
  /**
   * @brief Where the best loss must be stored
   */
  double& partialLoss;
  /**
   * @brief Where the split position associated to best loss must be stored
   */
  double& partialSplitPos;
  /**
   * @brief Function to colculate the loss itself for a given split
   * @see SAHKDTreeFactory::splitLoss
   */
  std::function<double(std::vector<Primitive*> const& primitives,
                       int const splitAxis,
                       double const splitPos,
                       double const r)>
    splitLoss;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Main constructor for SAH KDTree compute loss nodes sub-task
   */
  SAHKDTreeComputeLossNodesSubTask(
    std::shared_ptr<SharedTaskSequencer> ch,
    std::vector<Primitive*> const& primitives,
    double const start,
    double const step,
    int const splitAxis,
    double const minBound,
    double const boundLength,
    size_t const startNode,
    size_t const endNode,
    double& partialLoss,
    double& partialSplitPos,
    std::function<double(std::vector<Primitive*> const& primitives,
                         int const splitAxis,
                         double const splitPos,
                         double const r)> splitLoss)
    : SharedSubTask(ch)
    , primitives(primitives)
    , start(start)
    , step(step)
    , splitAxis(splitAxis)
    , minBound(minBound)
    , boundLength(boundLength)
    , startNode(startNode)
    , endNode(endNode)
    , partialLoss(partialLoss)
    , partialSplitPos(partialSplitPos)
    , splitLoss(splitLoss)
  {
  }
  ~SAHKDTreeComputeLossNodesSubTask() override {}

  // ***  RUNNABLE SHARED TASK  *** //
  // ****************************** //
  /**
   * @brief Implementation of the compute loss nodes method
   * @see SharedSubTask::run
   * @see SAHKDTreeFactory::findSplitPositionBySAH
   */
  void run() override
  {
    for (size_t i = startNode; i < endNode; ++i) {
      double const phi = start + ((double)i) * step;
      double const newLoss =
        splitLoss(primitives, splitAxis, phi, (phi - minBound) / boundLength);
      if (newLoss < partialLoss) {
        partialLoss = newLoss;
        partialSplitPos = phi;
      }
    }
  }
};
