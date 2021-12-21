#pragma once

#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Shared sub-task to populate splits when building a Simple KDTree.
 *  It is meant to be used at geometry-level parallelization context and only
 *  for right child nodes.
 * @see SharedSubTask
 * @see SimpleKDTreeFactory
 * @see MultiThreadKDTreeFactory
 *
 * TODO Rethink : Comment attributes, constructor and methods
 */
class SimpleKDTreePopulateSplitsSubTask : public SharedSubTask{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    vector<Primitive *> const &primitives;
    int const splitAxis;
    double const splitPos;
    vector<Primitive *> &leftPrimitives;
    vector<Primitive *> &rightPrimitives;
    size_t const startPrimitive;
    size_t const endPrimitive;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    SimpleKDTreePopulateSplitsSubTask(
        std::shared_ptr<SharedTaskSequencer> ch,
        vector<Primitive *> const &primitives,
        int const splitAxis,
        double const splitPos,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives,
        size_t const startPrimitive,
        size_t const endPrimitive
    ) :
        SharedSubTask(ch),
        primitives(primitives),
        splitAxis(splitAxis),
        splitPos(splitPos),
        leftPrimitives(leftPrimitives),
        rightPrimitives(rightPrimitives),
        startPrimitive(startPrimitive),
        endPrimitive(endPrimitive)
    {}
    virtual ~SimpleKDTreePopulateSplitsSubTask() {}

    // ***  RUNNABLE SHARED TASK  *** //
    // ****************************** //
    /**
     * @brief Implementation of the populate splits method
     * @see SharedSubTask::run
     */
    void run() override{
        for(size_t pi = startPrimitive ; pi < endPrimitive ; ++pi){
            Primitive *p = primitives[pi];
            // TODO Rethink : Common logic with populateSplits to common function
            AABB *box = p->getAABB();
            if(box->getMin()[splitAxis] <= splitPos)
                leftPrimitives.push_back(p);
            if(box->getMax()[splitAxis] > splitPos)
                rightPrimitives.push_back(p);
        }
    }
};