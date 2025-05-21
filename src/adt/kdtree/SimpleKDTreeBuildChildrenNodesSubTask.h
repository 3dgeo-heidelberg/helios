#pragma once

#include <KDTreeNode.h>
#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>

#include <functional>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Shared sub-task to build children nodes when building a Simple KDTree
 *  on all geometry-level depths except for the last one.
 * @see SharedSubTask
 * @see SimpleKDTreeFactory
 * @see MultiThreadKDTreeFactory
 */
class SimpleKDTreeBuildChildrenNodesSubTask : public SharedSubTask
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Node which children will be built if possible, otherwise it will
   *  be made a leaf node
   */
  KDTreeNode* node;
  /**
   * @brief Primitives of the node itself
   */
  vector<Primitive*>& primitives;
  /**
   * @brief Depth of current node
   */
  int const depth;
  /**
   * @brief Index of current node at current depth
   */
  int const index;
  /**
   * @brief Function to set child node
   */
  std::function<void(LightKDTreeNode*& child, KDTreeNode* node)> setChild;
  /**
   * @brief Function to recursively build children nodes
   */
  std::function<KDTreeNode*(KDTreeNode*,
                            bool const,
                            vector<Primitive*>&,
                            int const,
                            int const)>
    buildRecursive;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Main constructor for Simple KDTree build children nodes sub-task
   */
  SimpleKDTreeBuildChildrenNodesSubTask(
    std::shared_ptr<SharedTaskSequencer> ch,
    KDTreeNode* node,
    vector<Primitive*>& primitives,
    int const depth,
    int const index,
    std::function<void(LightKDTreeNode*& child, KDTreeNode* node)> setChild,
    std::function<KDTreeNode*(KDTreeNode*,
                              bool const,
                              vector<Primitive*>&,
                              int const,
                              int const)> buildRecursive)
    : SharedSubTask(ch)
    , node(node)
    , primitives(primitives)
    , depth(depth)
    , index(index)
    , setChild(setChild)
    , buildRecursive(buildRecursive)
  {
  }
  ~SimpleKDTreeBuildChildrenNodesSubTask() override {}

  // ***  RUNNABLE SHARED TASK  *** //
  // ****************************** //
  /**
   * @brief Implementation of the build children nodes method
   * @see SharedSubTask::run
   */
  void run() override
  {
    setChild(node->right,
             buildRecursive(node, false, primitives, depth + 1, 2 * index + 1));
  }
};
