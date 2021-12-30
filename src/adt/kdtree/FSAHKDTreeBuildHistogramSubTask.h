#pragma once

#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>
#include <surfaceinspector/maths/Histogram.hpp>

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Shared sub-task to build histogram from vertices when finding
 *  split position during Fast-SAH KDTree building. It is meant to be used at
 *  geometry-level parallelization context
 * @see SharedSubTask
 * @see FastSAHKDTreeFactory
 * @see FastSAHKDTreeFactory::findSplitPositionBySAH
 * @see MultiThreadKDTreeFactory
 */
class FSAHKDTreeBuildHistogramSubTask : public SharedSubTask {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Minimum coordinate of node boundary at corresponding split axis
     */
    double const minp;
    /**
     * @brief Maximum coordinate of node boundary at corresponding split axis
     */
    double const maxp;
    /**
     * @brief Vector of vertices coordinates representing node primitives.
     *  Each vertex is defined by its coordinate at corresponding split axis.
     * For the general case it is either the minimum coordinate or the maximum.
     */
    vector<double> &verts;
    /**
     * @brief How many bins the histogram must have
     */
    int const lossNodes;
    /**
     * @brief Where the histogram must be built
     */
    std::unique_ptr<Histogram<double>> &hist;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Main constructor for Fast-SAH KDTree build histogram from
     *  vertices sub-task
     */
    FSAHKDTreeBuildHistogramSubTask(
        std::shared_ptr<SharedTaskSequencer> ch,
        double const minp,
        double const maxp,
        vector<double> &verts,
        int const lossNodes,
        std::unique_ptr<Histogram<double>> &hist
    ) :
        SharedSubTask(ch),
        minp(minp),
        maxp(maxp),
        verts(verts),
        lossNodes(lossNodes),
        hist(hist)
    {}

    // ***  RUNNABLE SHARED TASK  *** //
    // ****************************** //
    /**
     * @brief Implementation of the method to build histogram from vertices
     * @see SharedSubTask::run
     * @see FastSAHKDTreeFactory::findSplitPositionBySAH
     */
    void run() override{
        hist = std::unique_ptr<Histogram<double>>(new Histogram<double>(
            minp, maxp, verts, lossNodes, false, false
        ));
    }
};