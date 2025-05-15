#pragma once

#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Shared sub-task to compute root node boundaries when building a
 *  Simple KDTree. It is meant to be used at geometry-level parallelization
 *  context and only for root node.
 * @see SharedSubTask
 * @see SimpleKDTreeFactory
 * @see SimpleKDTreeFactory::computeNodeBoundaries
 * @see MultiThreadKDTreeFactory
 */
class SimpleKDTreeComputeRootNodeBoundariesSubTask : public SharedSubTask
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Vector of primitives inside given root node
   */
  vector<Primitive*> const& primitives;
  /**
   * @brief Index of primitive (inclusive) at which the sub-task must start
   *  to iterate
   */
  size_t const startPrimitive;
  /**
   * @brief Index of primitive (exclusive) at which the sub-task must end
   *  iterating
   */
  size_t const endPrimitive;
  /**
   * @brief \f$x\f$ coordinate of min vertex
   */
  double& ax;
  /**
   * @brief \f$y\f$ coordinate of min vertex
   */
  double& ay;
  /**
   * @brief \f$z\f$ coordinate of min vertex
   */
  double& az;
  /**
   * @brief \f$x\f$ coordinate of max vertex
   */
  double& bx;
  /**
   * @brief \f$y\f$ coordinate of max vertex
   */
  double& by;
  /**
   * @brief \f$z\f$ coordinate of max vertex
   */
  double& bz;
  /**
   * @brief Function to digest a primitive when computing boundaries for
   *  root node
   */
  std::function<void(Primitive* primitive,
                     double& ax,
                     double& ay,
                     double& az,
                     double& bx,
                     double& by,
                     double& bz)>
    onRootBoundariesDigestPrimitive;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Main constructor for Simple KDTree compute root node boundaries
   *  sub-task
   */
  SimpleKDTreeComputeRootNodeBoundariesSubTask(
    std::shared_ptr<SharedTaskSequencer> ch,
    vector<Primitive*> const& primitives,
    size_t const startPrimitive,
    size_t const endPrimitive,
    double& ax,
    double& ay,
    double& az,
    double& bx,
    double& by,
    double& bz,
    std::function<void(Primitive* primitive,
                       double& ax,
                       double& ay,
                       double& az,
                       double& bx,
                       double& by,
                       double& bz)> onRootBoundariesDigestPrimitive)
    : SharedSubTask(ch)
    , primitives(primitives)
    , startPrimitive(startPrimitive)
    , endPrimitive(endPrimitive)
    , ax(ax)
    , ay(ay)
    , az(az)
    , bx(bx)
    , by(by)
    , bz(bz)
    , onRootBoundariesDigestPrimitive(onRootBoundariesDigestPrimitive)
  {
  }
  ~SimpleKDTreeComputeRootNodeBoundariesSubTask() override {}

  // ***  RUNNABLE SHARED TASK  *** //
  // ****************************** //
  /**
   * @brief Implementation of the compute  node boundaries method but for the
   *  root node only
   * @see SharedSubTask::run
   * @see SimpleKDTreeFactory::computeNodeBoundaries
   */
  void run() override
  {
    for (size_t pi = startPrimitive; pi < endPrimitive; ++pi) {
      onRootBoundariesDigestPrimitive(primitives[pi], ax, ay, az, bx, by, bz);
    }
  }
};
