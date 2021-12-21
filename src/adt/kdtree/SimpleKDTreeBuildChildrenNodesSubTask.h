#pragma once

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
 *
 * TODO Rethink : Comment attributes, constructor and methods
 */
class SimpleKDTreeBuildChildrenNodesSubTask : public SharedSubTask{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    KDTreeNode *node;
    vector<Primitive *> &primitives;
    int const depth;
    int const index;
    std::function<void(LightKDTreeNode *&child, KDTreeNode *node)> setChild;
    std::function<KDTreeNode *(
        KDTreeNode *,
        bool const,
        vector<Primitive *> &,
        int const,
        int const
    )> buildRecursive;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    SimpleKDTreeBuildChildrenNodesSubTask(
        std::shared_ptr<SharedTaskSequencer> ch,
        KDTreeNode *node,
        vector<Primitive *> &primitives,
        int const depth,
        int const index,
        std::function<void(LightKDTreeNode *&child, KDTreeNode *node)>setChild,
        std::function<KDTreeNode *(
            KDTreeNode *,
            bool const,
            vector<Primitive *> &,
            int const,
            int const
        )> buildRecursive
    ) :
        SharedSubTask(ch),
        node(node),
        primitives(primitives),
        depth(depth),
        index(index),
        setChild(setChild),
        buildRecursive(buildRecursive)
    {}
    virtual ~SimpleKDTreeBuildChildrenNodesSubTask() {}

    // ***  RUNNABLE SHARED TASK  *** //
    // ****************************** //
    /**
     * @brief Implementation of the build children nodes method
     * @see SharedSubTask::run
     */
    void run() override{
        setChild(node->right, buildRecursive(
            node, false, primitives, depth+1, 2*index+1
        ));
    }
};