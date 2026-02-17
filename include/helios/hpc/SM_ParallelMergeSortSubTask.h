#pragma once

#include <helios/surfaceinspector/maths/Scalar.hpp>
#include <helios/util/threadpool/SharedSubTask.h>
#include <helios/util/threadpool/SharedTaskSequencer.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace helios {
namespace hpc {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Shared sub-task to sort a sequence using a parallel mergesort like
 *  algorithm for shared memory contexts
 *
 * The parallel merge sort here implemented is based on spawning threads on
 *  a binary tree basis. For this purpose, let \f$n\f$ be the number of
 *  threads, so \f$\forall x \in [0, n-1],\; T_x\f$ notes the \f$x\f$-th
 *  thread. Let \f$d\f$ be the current depth and \f$d^*\f$ the max depth, so
 *  \f$d \in [0, d^*]\f$, where:
 *
 * \f[
 *  d^* = \left\lfloor{\log_2{(n)}}\right\rfloor
 * \f]
 *
 * For any \f$x\f$-thread it is possible to calculate its initial depth
 *  \f$d_*\f$, it is at which depth the thread was spwned:
 *
 * \f[
 *  d_* = \left\lceil\log_2{(x+1)}\right\rceil
 * \f]
 *
 * At each sub-task, the thread will take care of the left partition but it
 *  will delegate the right one to a new thread if possible. To do so, let
 *  \f$k=1+d-d_*\f$ for the \f$x-th\f$ thread. Thus, the \f$y\f$-thread to deal
 *  with right partition will be:
 *
 * \f[
 *  \begin{split}
 *      y(x, d) = \;& x + (2^k-1)x + 2^{k-1} \\
 *              = \;& 2^k x + 2^{k-1}
 *  \end{split}
 * \f]
 *
 * For instance, the \f$x=3\f$ thread at depth \f$d=4\f$ would correspond to
 *  the \f$y(3, 4) = 28\f$ thread. Thus, if \f$y(3, 4) = 28 < n\f$ is satisfied
 *  then a new thread will be spawned to handle that workload. Otherwise,
 *  the \f$x=3\f$ thread will have to sort the entire workload.
 *
 * As an example, a tree with \f$8\f$ threads would be:
 *
 * \f[
 * \left\{\begin{array}{lll}
 *  d=0 &:& \left\{
 *      T_0
 *  \right\} \\
 *  d=1 &:& \left\{
 *      T_0, T_1
 *  \right\} \\
 *  d=2 &:& \left\{
 *      T_0, T_2, T_1, T_3
 *  \right\} \\
 *  d=3 &:& \left\{
 *      T_0, T_4, T_2, T_5, T_1, T_6, T_3, T_7
 *  \right\}
 * \end{array}\right.
 * \f]
 *
 * Once a thread has finished its sorting, it waits for its immediate child to
 *  do the same. Next, the parent merges with child and so on recursively
 *  until the root node is reached. Merging process could have been implemented
 *  in a faster way by using both parent and child to merge, but this would
 *  require to work with a buffer of similar size than the sequence being
 *  sorted. This has been avoided relying the entire merge computation to
 *  each parent because this algorithm is meant to work with big sequences
 *  that might lead to out of memory scenarios.
 *
 * @see SharedSubTask
 * @see helios::hpc::SM_ParallelMergeSort
 */
template<typename RandomAccessIterator, typename Comparator>
class SM_ParallelMergeSortSubTask : public SharedSubTask
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @see helios::hpc::SM_ParallelMergeSort::stSequencer
   * @see SharedTaskSequencer
   */
  std::shared_ptr<SharedTaskSequencer> stSequencer;
  /**
   * @brief Thread index in \f$[0, n-1]\f$
   */
  size_t tIdx;
  /**
   * @see helios::hpc::SM_ParallelMergeSort::numThreads
   */
  size_t numThreads;
  /**
   * @see helios::hpc::SM_ParallelMergeSort::minElements
   */
  size_t minElements;
  /**
   * @see helios::hpc::SM_ParallelMergeSort::maxDepth
   */
  int maxDepth;
  /**
   * @see helios::hpc::SM_ParallelMergeSort::trySort
   */
  RandomAccessIterator begin;
  /**
   * @see helios::hpc::SM_ParallelMergeSort::trySort
   */
  RandomAccessIterator end;
  /**
   * @see helios::hpc::SM_ParallelMergeSort::trySort
   */
  Comparator comparator;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Main constructor for shared context parallel merge sort sub-task
   * @see helios::hpc::SM_ParallelMergeSort
   */
  SM_ParallelMergeSortSubTask(std::shared_ptr<SharedTaskSequencer> ch,
                              size_t const tIdx,
                              size_t const numThreads,
                              size_t const minElements,
                              int const maxDepth,
                              RandomAccessIterator begin,
                              RandomAccessIterator end,
                              Comparator comparator)
    : SharedSubTask(ch)
    , stSequencer(ch)
    , tIdx(tIdx)
    , numThreads(numThreads)
    , minElements(minElements)
    , maxDepth(maxDepth)
    , begin(begin)
    , end(end)
    , comparator(comparator)
  {
  }
  ~SM_ParallelMergeSortSubTask() override = default;

  // ***  RUNNABLE SHARED TASK  *** //
  // ****************************** //
  /**
   * @brief Implementation of the sort method itself
   * @see SharedSubTask::run
   * @see helios::hpc::SM_ParallelMergeSort
   * @see helios::hpc::SM_ParallelMergeSort::trySort
   */
  void run() override
  {
    // Handle single thread sort cases
    size_t const numElements = std::distance(begin, end); // m
    if (numElements < minElements || numThreads == 1) {
      std::sort(begin, end, comparator);
      return;
    }

    // Prepare handling of parallel cases
    int const initDepth = (int)std::ceil(std::log2(tIdx + 1)); // d_*
    RandomAccessIterator workA = begin;
    std::vector<RandomAccessIterator> workB(1, end);
    std::vector<std::shared_ptr<
      SM_ParallelMergeSortSubTask<RandomAccessIterator, Comparator>>>
      childrenTasks(0);

    // Compute workload distribution
    for (int depth = 0; depth < maxDepth; ++depth) {
      // Distribute workload to another thread if available
      size_t const k = 1 + depth - initDepth;
      size_t const rightIdx =
        (SurfaceInspector::maths::Scalar<size_t>::pow2(k)) * tIdx +
        SurfaceInspector::maths::Scalar<size_t>::pow2(k - 1);
      if (rightIdx < numThreads) {
        RandomAccessIterator workSplit =
          workA + std::distance(workA, workB[depth]) / 2;
        std::shared_ptr<SM_ParallelMergeSortSubTask> childTask =
          std::make_shared<
            SM_ParallelMergeSortSubTask<RandomAccessIterator, Comparator>>(
            stSequencer,
            rightIdx,
            numThreads,
            minElements,
            maxDepth,
            workSplit,
            workB[depth],
            comparator);
        childrenTasks.push_back(childTask);
        stSequencer->start(childTask);
        workB.push_back(workSplit);
      } else {
        break;
      }
    }

    // Compute workload itself (partial sorting)
    std::sort(workA, workB[workB.size() - 1], comparator);

    // Merge computed workload (merge partial sortings)
    for (int i = childrenTasks.size() - 1; i >= 0; --i) {
      std::shared_ptr<SM_ParallelMergeSortSubTask> childTask = childrenTasks[i];
      childTask->getThread()->join();
      std::inplace_merge(workA, workB[i + 1], workB[i], comparator);
    }
    return;
  }
};

}
}
