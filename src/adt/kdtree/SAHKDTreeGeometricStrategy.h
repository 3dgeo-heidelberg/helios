#pragma once

#include <SAHKDTreeFactory.h>
#include <SimpleKDTreeGeometricStrategy.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing a strategy for geometry-level parallelization of
 *  SAH KDTree building. The geometry-level parallelization is the one to be
 *  applied on upper tree nodes, where multiple threads work on the building
 *  of the same node.
 *
 * @see SimpleKDTreeGeometricStrategy
 */
class SAHKDTreeGeometricStrategy : public SimpleKDTreeGeometricStrategy{
    // ***  FRIENDS  *** //
    // ***************** //
    friend class MultiThreadKDTreeFactory;

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The surface area heuristic KDTree factory to which geometric
     *  strategy shall be applied
     */
    SAHKDTreeFactory &sahkdtf;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief SAHKDTreeGeometricStrategy default constructor.
     *
     * @see SAHKDTreeGeoemtricStrategy::sahkdtf
     * @see SimpleKDTreeGeometricStrategy::SimpleKDTreeGeometricStrategy
     */
    SAHKDTreeGeometricStrategy(SAHKDTreeFactory &kdtf):
        SimpleKDTreeGeometricStrategy(kdtf),
        sahkdtf(kdtf)
    {}
    ~SAHKDTreeGeometricStrategy() override = default;

    // ***  CLONE  *** //
    // *************** //
    /**
     * @brief Create a clone of the SAHKDTreeGeometricStrategy
     * @param kdtf The KDTreeFactory to be referenced by the clone
     * @return Clone of the SAHKDTreeGeometricStrategy
     */
    SimpleKDTreeGeometricStrategy * clone(
        SimpleKDTreeFactory *kdtf
    ) const override;

protected:
    // ***  GEOMETRY LEVEL BUILDING  *** //
    // ********************************* //
    /**
     * @brief Geometry-level parallel version of the
     *  SAHKDTreeFactory::defineSplit function
     *
     * @see SAHKDTreeFactory::defineSplit
     * @see SimpleKDTreeGeometricStrategy::GEOM_defineSplit
     */
    void GEOM_defineSplit(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> &primitives,
        int const depth,
        int const assignedThreads
    ) const override;
    /**
     * @brief Geometry-level parallel version of the
     *  SAHKDTreeFactory::buildChildrenNodes
     *
     * @see SAHKDTreeFactory::buildChildrenNodes
     * @see SimpleKDTreeGeometricStrategy::GEOM_buildChildrenNodes
     */
    void GEOM_buildChildrenNodes(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> const &primitives,
        int const depth,
        int const index,
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives,
        std::shared_ptr<SharedTaskSequencer> masters
    ) override;
    /**
     * @brief Geometry-level parallel version of the
     *  SAHKDTreeFactory::findSplitPositionBySAH function
     *
     * @param assignedThreads How many threads can be used to parallelize
     *  computations
     * @see SAHKDTreeFactory::findSplitPositionBySAH
     */
    virtual double GEOM_findSplitPositionBySAH(
        KDTreeNode *node,
        vector<Primitive *> &primitives,
        int assignedThreads
    ) const;
};