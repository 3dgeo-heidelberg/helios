#pragma once

#include <SAHKDTreeGeometricStrategy.h>
#include <FastSAHKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing a strategy for geometry-level parallelization of
 *  Fast SAH KDTree building. The geometry-level parallelization is the one to
 *  be applied on upper tree nodes, where multiple threads work on the
 *  building of the same node.
 *
 * @see SAHKDTreeGeometricStrategy
 */
class FastSAHKDTreeGeometricStrategy : public SAHKDTreeGeometricStrategy {
    // ***  FRIENDS  *** //
    // ***************** //
    friend class MultiThreadKDTreeFactory;

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The fast surface area heuristic KDTree factory to which geometric
     *  strategy shall be applied
     */
    FastSAHKDTreeFactory &fsahkdtf;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief FastSAHKDTreeGeometricStrategy default constructor.
     *
     * @see FastSAHKDTreeGeometricStrategy::fsahkdtf
     * @see SAHKDTreeGeometricStrategy::SAHKDTreeGeometricStrategy
     */
    FastSAHKDTreeGeometricStrategy(FastSAHKDTreeFactory &kdtf) :
        SAHKDTreeGeometricStrategy(kdtf),
        fsahkdtf(kdtf)
    {}
    virtual ~FastSAHKDTreeGeometricStrategy() = default;

    // ***  GEOMETRY LEVEL BUILDING  *** //
    // ********************************* //
    /**
     * @brief Geometry-level parallel version of the
     *  FastSAHKDTreeFactory::findSplitPositionBySAH function
     *
     * @param assignedThreads How many threads can be used to parallelize
     *  computations
     * @see FastSAHKDTreeFactory::findSplitPositionBySAH
     */
    double GEOM_findSplitPositionBySAH(
        KDTreeNode *node,
        vector<Primitive *> &primitives,
        int assignedThreads
    ) const override;
};