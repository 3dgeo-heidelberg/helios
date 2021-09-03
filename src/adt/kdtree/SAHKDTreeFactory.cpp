#include <SAHKDTreeFactory.h>

#include <KDTreePrimitiveComparator.h>

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
    Primitive * phi = primitives[m/2]; // The median object
    // TODO Rethink : This is the Simple KDTree implementation. ...
    // ... It must be substituted by the SAH implementation
    splitPos = phi->getCentroid()[splitAxis];
};
