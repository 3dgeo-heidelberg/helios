#include <FastSAHKDTreeGeometricStrategy.h>
#include <FSAHKDTreeExtractMinMaxVerticesSubTask.h>
#include <FSAHKDTreeBuildHistogramSubTask.h>


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
            int const splitAxis,
            double const minp,
            double const maxp,
            vector<Primitive *> &primitives,
            vector<double> &minVerts,
            vector<double> &maxVerts
        ) -> void { // Extract min-max vertices
            // Distribute workload
            size_t const numPrimitives = primitives.size();
            /*
             * Using assignedThreads = min(assignedThreads, numPrimitives)
             *  might degrade performance because the overhead of handling thread
             *  contexts could overcome the improvement from parallelization.
             * If this is detected, switching strategy to something like
             *  assignedThreads = min(assignedThreads, numPrimitives/k), for k > 1,
             *  might alleviate the problem with an adequate k.
             * However, this is expected to be unlikely because geometry-level
             *  parallelization is applied on upper tree nodes, where the number
             *  of primitives stills high.
             */
            if(assignedThreads > (int)numPrimitives)
                assignedThreads = (int)numPrimitives;
            size_t const chunkSize = numPrimitives / ((size_t)assignedThreads);
            int const extraThreads = assignedThreads - 1;
            std::shared_ptr<SharedTaskSequencer> stSequencer = \
                std::make_shared<SharedTaskSequencer>(extraThreads);
            vector<vector<double>> localMinVerts(assignedThreads);
            vector<vector<double>> localMaxVerts(assignedThreads);
            for(int i = 0 ; i < extraThreads ; ++i){
                stSequencer->start(
                    std::make_shared<FSAHKDTreeExtractMinMaxVerticesSubTask>(
                        stSequencer,
                        splitAxis,
                        minp,
                        maxp,
                        primitives,
                        localMinVerts[i],
                        localMaxVerts[i],
                        i*chunkSize,
                        (i+1)*chunkSize
                    )
                );
            }
            FSAHKDTreeExtractMinMaxVerticesSubTask(
                nullptr,
                splitAxis,
                minp,
                maxp,
                primitives,
                localMinVerts[extraThreads],
                localMaxVerts[extraThreads],
                extraThreads*chunkSize,
                numPrimitives
            )();

            // Wait until workload has been consumed
            stSequencer->joinAll();

            // Merge all local vertices together
            for(int i = 0 ; i < assignedThreads ; ++i){
                minVerts.insert(
                    minVerts.end(),
                    localMinVerts[i].begin(),
                    localMinVerts[i].end()
                );
                maxVerts.insert(
                    maxVerts.end(),
                    localMaxVerts[i].begin(),
                    localMaxVerts[i].end()
                );
            }
        },
        [&] (
            double const minp,
            double const maxp,
            vector<double> &minVerts,
            vector<double> &maxVerts,
            int const lossNodes,
            std::unique_ptr<Histogram<double>> &Hmin,
            std::unique_ptr<Histogram<double>> &Hmax
        ) -> void { // Build histograms
            std::shared_ptr<SharedTaskSequencer> stSequencer = \
                std::make_shared<SharedTaskSequencer>(1);
            stSequencer->start(std::make_shared<
                FSAHKDTreeBuildHistogramSubTask
            >(
                stSequencer, minp, maxp, minVerts, lossNodes, Hmin
            ));
            Hmax = std::unique_ptr<Histogram<double>>(new Histogram<double>(
                minp, maxp, maxVerts, lossNodes, false, false
            ));
            stSequencer->joinAll();
        }
    );
    // TODO Rethink : Use std::async for histograms instead of pointer subtask?
}
