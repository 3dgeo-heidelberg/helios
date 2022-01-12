#pragma once

#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>

class FastSAHKDTreeRecountSubTask : public SharedSubTask{
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
    double const minp;
    /**
     * @brief Difference between maximum and minimum coordinates of node
     *  boundaries at corresponding split axis
     */
    double const deltap;
    /**
     * @brief First primitive to be considered by the recount (inclusive)
     */
    vector<Primitive *>::iterator beginPrimitive;
    /**
     * @brief Last primitive to be considered by the recount (exclusive)
     */
    vector<Primitive *>::iterator endPrimitive;
    /**
     * @brief How many bins use to cound
     */
    size_t const lossNodes;
    /**
     * @brief How many forward and backward count cases (it is, bins + 1)
     */
    size_t const lossCases;
    /**
     * @brief Where forward count components must be stored
     */
    vector<size_t> &cForward;
    /**
     * @brief Where backward count components must be stored
     */
    vector<size_t> &cBackward;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Main constructor for Fast-SAH KDTree forward and backward
     *  recounts
     */
    FastSAHKDTreeRecountSubTask(
        std::shared_ptr<SharedTaskSequencer> ch,
        int const splitAxis,
        double const minp,
        double const deltap,
        vector<Primitive *>::iterator beginPrimitive,
        vector<Primitive *>::iterator endPrimitive,
        size_t const lossNodes,
        size_t const lossCases,
        vector<size_t> &cForward,
        vector<size_t> &cBackward
    ) :
        SharedSubTask(ch),
        splitAxis(splitAxis),
        minp(minp),
        deltap(deltap),
        beginPrimitive(beginPrimitive),
        endPrimitive(endPrimitive),
        lossNodes(lossNodes),
        lossCases(lossCases),
        cForward(cForward),
        cBackward(cBackward)
    {}
    virtual ~FastSAHKDTreeRecountSubTask() = default;

    // ***  RUNNABLE SHARED TASK  *** //
    // ****************************** //
    /**
     * @brief Implementation of the method to do forward and backward recount
     * @see SharedSubTask::run
     * @see FastSAHKDTreeFactory::findSplitPositionBySAH
     */
    void run() override{
        // Count min and max vertices
        vector<size_t> minCount(lossNodes, 0);
        vector<size_t> maxCount(lossNodes, 0);
        for(
            vector<Primitive *>::iterator currentPrimitive = beginPrimitive;
            currentPrimitive < endPrimitive;
            ++currentPrimitive
        ){
            AABB *aabb = (*currentPrimitive)->getAABB();
            double const minq = aabb->getMin()[splitAxis];
            double const maxq = aabb->getMax()[splitAxis];
            ++minCount[
                std::min<size_t>(
                    (size_t)((minq-minp)/deltap * lossNodes),
                    lossNodes-1
                )
            ];
            ++maxCount[
                std::min<size_t>(
                    (size_t)((maxq-minp)/deltap * lossNodes),
                    lossNodes-1
                )
            ];
        }

        // Accumulate counts
        cForward.front() = 0;
        for(size_t i = 0 ; i < lossNodes ; ++i){
            cForward[i+1] = cForward[i] + minCount[i];
        }
        cBackward.back() = 0;
        for(size_t i = lossNodes ; i > 0 ; --i){
            cBackward[i-1] = cBackward[i] + maxCount[lossNodes-i];
        }

    }

};