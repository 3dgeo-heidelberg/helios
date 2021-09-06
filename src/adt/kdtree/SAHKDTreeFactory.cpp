#include <SAHKDTreeFactory.h>

#include <KDTreePrimitiveComparator.h>
#include <fluxionum/UnivariateNewtonRaphsonMinimizer.h>

using fluxionum::UnivariateNewtonRaphsonMinimizer;

// ***  BUILDING METHODS  *** //
// ************************** //
void SAHKDTreeFactory::defineSplit(
    vector<Primitive *> &primitives,
    int const depth,
    int &splitAxis,
    double &splitPos
) const {
    // Find split axis
    splitAxis = depth % 3;

    // Obtain the object median
    size_t const m = primitives.size();
    std::sort(
        primitives.begin(),
        primitives.end(),
        KDTreePrimitiveComparator(splitAxis)
    );
    Primitive * medianObject = primitives[m/2]; // The median object
    double const objectMedian = medianObject->getCentroid()[splitAxis];

    // Obtain the spatial median
    // TODO Rethink : If f=df use newton raphson for roots and not minimization
    UnivariateNewtonRaphsonMinimizer<double, double> unrm(
        [&primitives, &splitAxis] (double phi) -> double {
            double dist = 0.0;
            for(Primitive *primitive : primitives)
                dist += std::fabs(primitive->getCentroid()[splitAxis]-phi);
            return dist;
        },
        [&primitives, &splitAxis] (double phi) -> double {
            double dist = 0.0;
            for(Primitive *primitive : primitives)
                dist += std::fabs(primitive->getCentroid()[splitAxis]-phi);
            return dist;
        },
        [&primitives, &splitAxis] (double phi) -> double {
            double diff = 0.0;
            for(Primitive *primitive : primitives){
                double const p = primitive->getCentroid()[splitAxis];
                if(p!=phi) diff += (phi-p)/std::fabs(p-phi);
            }
            return diff;
        }
    );
    // TODO Rethink : This is the centroid between spatial and object medians
    // ... It must be substituted by the full SAH implementation
    splitPos = unrm.argmin(objectMedian)/2.0;

    // TODO Rethink : This is the Simple KDTree implementation. ...
    // ... It must be substituted by the SAH implementation
    splitPos = medianObject->getCentroid()[splitAxis];
};
