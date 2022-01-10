#pragma once

#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Shared sub-task to extract minimum and maximum vertices when finding
 *  split position during Fast-SAH KDTree building. It is meant to be used at
 *  geometry-level parallelization context
 * @see SharedSubTask
 * @see FastSAHKDTreeFactory
 * @see FastSAHKDTreeFactory::findSplitPositionBySAH
 * @see MultiThreadKDTreeFactory
 */
class FSAHKDTreeExtractMinMaxVerticesSubTask : public SharedSubTask {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The node split axis
     */
    int const splitAxis;
    /**
     * @brief Minimum coordinate of node boundary at corresponding split axis
     */
    double minp;
    /**
     * @brief Maximum coordinate of node boundary at corresponding split axis
     */
    double maxp;
    /**
     * @brief Primitives belonging to the node
     */
    vector<Primitive *> &primitives;
    /**
     * @brief Vector of minimum vertices to be populated from node primitives
     *  Each vertex is defined by its coordinate at corresponding split axis.
     */
    vector<double> &minVerts;
    /**
     * @brief Vector of maximum vertices to be populated from node primitives.
     *  Each vertex is defined by its coordinate at corresponding split axis.
     */
    vector<double> &maxVerts;
    /**
     * @brief Index of start primitive (inclusive) that must be digested by
     *  this sub-task
     */
    size_t const startPrimitive;
    /**
     * @brief Index of end primitive (exclusive) that must be digested by
     *  this sub-task
     */
    size_t const endPrimitive;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Main constructor for Fast-SAH KDTree extract min and max vertices
     *  sub-task
     */
    FSAHKDTreeExtractMinMaxVerticesSubTask(
        std::shared_ptr<SharedTaskSequencer> ch,
        int const splitAxis,
        double const minp,
        double const maxp,
        vector<Primitive *> &primitives,
        vector<double> &minVerts,
        vector<double> &maxVerts,
        size_t const startPrimitive,
        size_t const endPrimitive
    ) :
        SharedSubTask(ch),
        splitAxis(splitAxis),
        minp(minp),
        maxp(maxp),
        primitives(primitives),
        minVerts(minVerts),
        maxVerts(maxVerts),
        startPrimitive(startPrimitive),
        endPrimitive(endPrimitive)
    {}
    virtual ~FSAHKDTreeExtractMinMaxVerticesSubTask() = default;

    // ***  RUNNABLE SHARED TASK  *** //
    // ****************************** //
    /**
     * @brief Implementation of the method to extract min and max vertices
     * @see SharedSubTask::run
     * @see FastSAHKDTreeFactory::findSplitPositionBySAH
     */
    void run() override{
        for(size_t i = startPrimitive ; i < endPrimitive ; ++i){
            AABB *aabb = primitives[i]->getAABB();
            double const minq = aabb->getMin()[splitAxis];
            double const maxq = aabb->getMax()[splitAxis];
            minVerts[i] = (minq < minp) ? minp : minq;
            maxVerts[i] = (maxq > maxp) ? maxp : maxq;
        }
    }
};