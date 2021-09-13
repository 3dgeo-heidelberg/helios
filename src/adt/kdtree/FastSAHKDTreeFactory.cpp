#include <FastSAHKDTreeFactory.h>
#include <surfaceinspector/maths/Histogram.hpp>

using SurfaceInspector::maths::Histogram;

// ***  SAH UTILS  *** //
// ******************* //
double FastSAHKDTreeFactory::splitLoss(
    vector<Primitive *> const &primitives,
    int const splitAxis,
    double const splitPos,
    double const r
) const {
    // Split in left and right primitives
    size_t lps = 0, rps = 0;
    for(Primitive *primitive : primitives){
        AABB const *primBox = primitive->getAABB();
        if(primBox->getMin()[splitAxis] <= splitPos) ++lps;
        if(primBox->getMax()[splitAxis] > splitPos) ++rps;
    }

    // Compute and return loss function
    return r*((double)lps) + (1.0-r)*((double)rps);
}

double FastSAHKDTreeFactory::findSplitPositionBySAH(
    KDTreeNode *node,
    vector<Primitive *> &primitives
) const {
    // Extract centroids
    vector<double> centroids;
    for(Primitive *primitive : primitives){
        centroids.push_back(primitive->getCentroid()[node->splitAxis]);
    }
    Histogram<double> H(centroids, numBins, true, false);

    // Approximated discrete search of optimal splitting plane
    double const a = node->bound.getMin()[node->splitAxis];
    double const b = node->bound.getMax()[node->splitAxis];
    double const length = b-a;
    double const mu = (b+a)/2.0;
    double me = H.findCutPoint(0.5);
    if(me < a) me = a;
    if(me > b) me = b;
    double const start = (mu>me) ? me : mu;
    double const step = (mu>me) ? (mu-me)/((double)(lossNodes-1)) :
        (me-mu)/((double)(lossNodes-1));
    double loss = std::numeric_limits<double>::max();
    for(size_t i = 0 ; i < lossNodes ; ++i){
        double const phi = start + ((double)i)*step;
        double const newLoss = splitLoss(
            primitives,
            node->splitAxis,
            phi,
            (phi-a) / length
        );
        if(newLoss < loss){
            loss = newLoss;
            node->splitPos = phi;
        }
    }

    // Store loss if requested
    return loss;

}
