#pragma once

#include <SAHKDTreeGeometricStrategy.h>
#include <AxisSAHKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing a strategy for geometry-level parallelization of
 *  Axis SAH KDTree building. The geometry-level parallelization is the one to
 *  be applied on upper tree nodes, where multiple threads work on the
 *  building of the same node.
 *
 * @see SAHKDTreeGeometricStrategy
 */
class AxisSAHKDTreeGeometricStrategy : public SAHKDTreeGeometricStrategy {
    // ***  FRIENDS  *** //
    // ***************** //
    friend class MultiThreadKDTreeFactory;

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The axis surface area heuristic KDTree factory to which geometric
     *  strategy shall be applied
     */
    AxisSAHKDTreeFactory &asahkdtf;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief AxisSAHKDTreeGeometricStrategy default constructor.
     *
     * @see AxisSAHKDTreeGeometricStrategy::asahkdtf
     * @see SAHKDTreeGeometricStrategy::SAHKDTreeGeometricStrategy
     */
    AxisSAHKDTreeGeometricStrategy(AxisSAHKDTreeFactory &kdtf):
        SAHKDTreeGeometricStrategy(kdtf),
        asahkdtf(kdtf)
    {}
    virtual ~AxisSAHKDTreeGeometricStrategy() = default;

protected:
    // ***  GEOMETRY LEVEL BUILDING  *** //
    // ********************************* //
    /**
     * @brief Extend SAHKDTreeFactory::GEOM_defineSplit to handle greedy
     *  search of best split axis inside a geometry-level parallelization
     *  context
     * @see SAHKDTreeFactory::GEOM_defineSplit
     */
    void GEOM_defineSplit(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> &primitives,
        int const depth,
        int const assignedThreads
    ) const override;
};