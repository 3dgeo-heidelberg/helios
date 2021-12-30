#include <FastSAHKDTreeFactory.h>
#include <FSAHKDTreeExtractMinMaxVerticesSubTask.h>
#include <FSAHKDTreeBuildHistogramSubTask.h>

using SurfaceInspector::maths::Histogram;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
FastSAHKDTreeFactory::FastSAHKDTreeFactory(
    size_t const lossNodes,
    double const ci,
    double const cl,
    double const co
) : SAHKDTreeFactory(lossNodes, ci, cl, co)
{}

// ***  SAH UTILS  *** //
// ******************* //
double FastSAHKDTreeFactory::findSplitPositionBySAH(
    KDTreeNode *node,
    vector<Primitive *> &primitives
) const {
    return findSplitPositionByFastSAHRecipe(
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
            for(Primitive *q : primitives){
                double const minq = q->getAABB()->getMin()[splitAxis];
                double const maxq = q->getAABB()->getMax()[splitAxis];
                minVerts.push_back((minq < minp) ? minp : minq);
                maxVerts.push_back((maxq > maxp) ? maxp : maxq);
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
            Hmin = std::unique_ptr<Histogram<double>>(new Histogram<double>(
                minp, maxp, minVerts, lossNodes, false, false
            ));
            Hmax = std::unique_ptr<Histogram<double>>(new Histogram<double>(
                minp, maxp, maxVerts, lossNodes, false, false
            ));
        }
    );
}

double FastSAHKDTreeFactory::findSplitPositionByFastSAHRecipe(
    KDTreeNode *node,
    vector<Primitive *> &primitives,
    std::function<void(
        int const splitAxis,
        double const minp,
        double const maxp,
        vector<Primitive *> &primitives,
        vector<double> &minVerts,
        vector<double> &maxVerts
    )> f_extractMinMaxVertices,
    std::function<void(
        double const minp,
        double const maxp,
        vector<double> &minVerts,
        vector<double> &maxVerts,
        int const lossNodes,
        std::unique_ptr<Histogram<double>> &Hmin,
        std::unique_ptr<Histogram<double>> &Hmax
    )> f_buildHistograms
) const {
    /*
     * Code below is commented because it might be necessary in the future.
     * If problems are found with the fast SAH implementation, notice that it
     *  is a well known issue that the min-max method might cause problems at
     *  nodes where number of primitives is smaller than the number of bins.
     *  In such cases, it is recommended to use the full SAH computation
     *  instead of the min-max approximation.
     *
     * If performance is not good enough, instead of using lossNodes for the
     *  full SAH and the fast SAH, implement a numBins attribute for the
     *  fast SAH. Then, use lossNodes just for full SAH. Good initial values
     *  for the average case would be 21 lossNodes for full SAH and 32 numBins
     *  for the fast SAH
     */
    // If there are not enough primitives, use a more accurate loss computation
    /*if(primitives.size() <= numBins)
        return SAHKDTreeFactory::findSplitPositionBySAH(node, primitives);*/

    // Extract min and max vertices in the same discrete space
    vector<double> minVerts, maxVerts;
    double const minp = node->bound.getMin()[node->splitAxis];
    double const maxp = node->bound.getMax()[node->splitAxis];
    f_extractMinMaxVertices(
        node->splitAxis, minp, maxp, primitives, minVerts, maxVerts
    );
    std::unique_ptr<Histogram<double>> _Hmin = nullptr;
    std::unique_ptr<Histogram<double>> _Hmax = nullptr;
    f_buildHistograms(
        minp, maxp, minVerts, maxVerts, lossNodes, _Hmin, _Hmax
    );
    Histogram<double> &Hmin = *_Hmin;
    Histogram<double> &Hmax = *_Hmax;

    // Approximated discrete search of optimal splitting plane
    size_t NoLr = 0;
    size_t NoRr = primitives.size();
    double loss = (double) NoRr, newLoss;
    node->splitPos = Hmin.a[0];
    double const rDenom = (double) lossNodes;
    for(size_t i = 1 ; i <= lossNodes ; ++i){
        double const r = ((double)i) / rDenom;
        NoLr += Hmin.c[i-1];
        NoRr -= Hmax.c[i-1];
        newLoss = r*((double)NoLr) + (1.0-r)*((double)NoRr);
        if(newLoss < loss){
            loss = newLoss;
            node->splitPos = Hmin.b[i-1];
        }
    }

    // Store loss if requested
    return loss;
}

// ***  GEOMETRY LEVEL BUILDING  *** //
// ********************************* //
double FastSAHKDTreeFactory::GEOM_findSplitPositionBySAH(
    KDTreeNode *node,
    vector<Primitive *> &primitives,
    int assignedThreads
) const {
    // Handle cases where sequential execution is preferred
    if(assignedThreads==1 || ((int)primitives.size()) < 2*assignedThreads){
        return findSplitPositionBySAH(node, primitives);
    }
    // Handle multi-thread cases
    return findSplitPositionByFastSAHRecipe(
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
}
