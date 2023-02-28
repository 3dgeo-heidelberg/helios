#include <FastSAHKDTreeGeometricStrategy.h>
#include <FastSAHKDTreeRecountSubTask.h>

// ***  CLONE  *** //
// *************** //
SimpleKDTreeGeometricStrategy * FastSAHKDTreeGeometricStrategy::clone(
    SimpleKDTreeFactory *kdtf
) const {
    return new FastSAHKDTreeGeometricStrategy(*((FastSAHKDTreeFactory *)kdtf));
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
double FastSAHKDTreeGeometricStrategy::GEOM_findSplitPositionBySAH(
    KDTreeNode *node,
    vector<Primitive *> &primitives,
    int assignedThreads
) const {
    // Handle cases where sequential execution is preferred
    if(assignedThreads==1 || ((int)primitives.size()) < 2*assignedThreads){
        return fsahkdtf.findSplitPositionBySAH(node, primitives);
    }
    // Handle multi-thread cases
    return fsahkdtf.findSplitPositionByFastSAHRecipe(
        node,
        primitives,
        [&] (
            vector<Primitive *> &primitives,
            int const splitAxis,
            double const minp,
            double const deltap,
            size_t const lossNodes,
            size_t const lossCases,
            vector<size_t> &cForward,
            vector<size_t> &cBackward
        ) -> void { // Count forward and backward
            // Distribute workload
            size_t const numPrimitives = primitives.size();
            /*
             * Using assignedThreads = min(assignedThreads, numPrimitives)
             *  might degrade performance because the overhead of handling
             *  thread contexts could overcome the improvement from
             *  parallelization.
             * If this is detected, switching strategy to something like
             *  assignedThreads = min(assignedThreads, numPrimitives/k),
             *  for k > 1, might alleviate the problem with an adequate k.
             * However, this is expected to be unlikely because geometry-level
             *  parallelization is applied on upper tree nodes, where the
             *  number of primitives stills high.
             */
            if(assignedThreads > (int)numPrimitives)
                assignedThreads = (int)numPrimitives;
            size_t const chunkSize = numPrimitives / ((size_t)assignedThreads);
            vector<vector<size_t>> lForward(
                assignedThreads,
                vector<size_t>(lossCases, 0)
            );
            vector<vector<size_t>> lBackward = lForward;
            int const extraThreads = assignedThreads - 1;
            std::shared_ptr<SharedTaskSequencer> stSequencer = \
                std::make_shared<SharedTaskSequencer>(extraThreads);
            // Parallel compute auxiliar threads
            for(int i = 0 ; i < extraThreads ; ++i){
                stSequencer->start(
                    std::make_shared<FastSAHKDTreeRecountSubTask>(
                        stSequencer,
                        splitAxis,
                        minp,
                        deltap,
                        primitives.begin()+i*chunkSize,
                        primitives.begin()+(i+1)*chunkSize,
                        lossNodes,
                        lossCases,
                        lForward[i],
                        lBackward[i]
                    )
                );
            }
            // Main thread computation
            FastSAHKDTreeRecountSubTask(
                nullptr,
                splitAxis,
                minp,
                deltap,
                primitives.begin()+extraThreads*chunkSize,
                primitives.end(),
                lossNodes,
                lossCases,
                lForward.back(),
                lBackward.back()
            )();

            // Wait until workload has been consumed
            stSequencer->joinAll();

            // Reduce local counts to counts
            for(size_t i = 0 ; i < lossCases ; ++i){
                for(vector<size_t> &lf : lForward){
                    cForward[i] += lf[i];
                }
                for(vector<size_t> &lb : lBackward){
                    cBackward[i] += lb[i];
                }
            }
        }
    );
}
