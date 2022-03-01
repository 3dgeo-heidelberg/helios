#pragma once

#include <SAHKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for k-dimensional trees with surface
 *  area heuristic and greedy selection of best partition axis
 *
 * @see SAHKDTreeFactory
 */
class AxisSAHKDTreeFactory : public SAHKDTreeFactory{
using SAHKDTreeFactory::lossNodes;
using SAHKDTreeFactory::ci;
using SAHKDTreeFactory::cl;
using SAHKDTreeFactory::co;
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize an axis surface area heuristic KDTree factory to a
     *  stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the surface area heuristic KDTree
     *  factory
     */
    template <typename Archive>
    void serialize(Archive &ar, unsigned int const version){
        boost::serialization::void_cast_register<
            AxisSAHKDTreeFactory,
            SAHKDTreeFactory
        >();
        ar &boost::serialization::base_object<SAHKDTreeFactory>(*this);
    }

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Axis surface area heuristic KDTree factory default constructor
     * @see SAHKDTreeFactory::SAHKDTreeFactory
     */
    AxisSAHKDTreeFactory (
        size_t const lossNodes=21,
        double const ci=1,
        double const cl=1,
        double const co=1
    );
    virtual ~AxisSAHKDTreeFactory() = default;

    // ***  CLONE  *** //
    // *************** //
    /**
     * @see KDTreeFactory::clone
     */
    KDTreeFactory * clone() const override;
    /**
     * @brief Assign attributes from AxisSAHKDTreeFactory to its clone
     */
    void _clone(KDTreeFactory *kdtf) const override;

    // ***  BUILDING METHODS  *** //
    // ************************** //
    /**
     * @brief Extend SAHKDTreeFactory::defineSplit to handle greedy search of
     *  best split axis
     * @see SAHKDTreeFactory::defineSplit
     */
    void defineSplit(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> &primitives,
        int const depth
    ) const override;

};