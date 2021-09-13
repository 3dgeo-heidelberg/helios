#pragma once

#include <SAHKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for k-dimensional trees with a
 *  fast strategy to approximate Surface Area Heuristic (SAH)
 *
 *
 * The fast strategy to approximate Surface Area Heuristic (SAH) is based on
 *  minmax histograms. Thus let
 *  \f$H_* = \left\{
 *      [a_1, b_1), \ldots, [a_{m-1}, b_{m-1}), [a_m, b_m]
 *  \right\}\f$ be the min histogram and
 *  \f$H^* = \left\{
 *      [c_1, d_1), \ldots, [c_{m-1}, d_{m-1}), [c_m, d_m]
 *  \right\}\f$ be the max histogram. Where each bin is defined by its start
 *  point (\f$a_i\f$ for \f$H_*\f$ and \f$c_i\f$ for \f$H^*\f$) and its end
 *  point (\f$b_i\f$ for \f$H_*\f$ and \f$d_i\f$ for \f$H^*\f$). Thus, the
 *  number of objects per part can be redefined as follows:
 *
 * \f[
 * \left\{\begin{array}{lll}
 *  N_o(L_r) &=& \left|\left\{x : x \leq p_* \right\}\right| \\
 *  N_o(R_r) &=& \left|\left\{x : x \geq p^* \right\}\right|
 * \end{array}\right.
 * \f]
 *
 * Where \f$p_* = b_i\f$ for which
 *  \f$\mathrm{argmin}_i \left|\frac{a_i+b_i}{2} - \varphi(t)\right|\f$ and
 *  \f$p^* = c_i\f$ for which
 *  \f$\mathrm{argmin}_i \left|\frac{c_i+d_i}{2} - \varphi(t)\right|\f$.
 *  Understanding \f$\varphi(t)\f$ as described for the iterative method of
 *  SAHKDTreeFactory::defineSplit.
 *
 * Thus, by means of modifying how the loss function is computed, sorting is
 *  no longer required and can be replaced by a cheap approximation based on
 *  min-max method.
 *
 * @see SAHKDTreeFactory::defineSplit
*/

class FastSAHKDTreeFactory : public SAHKDTreeFactory {
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a fast surface area heuristic KDTree factory to a
     *  stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the fast surface area heuristic KDTRee
     *  factory
     */
    template <typename Archive>
    void serialize(Archive &ar, unsigned int const version){
        boost::serialization::void_cast_register<
            FastSAHKDTreeFactory,
            SAHKDTreeFactory
        >();
        ar &boost::serialization::base_object<SAHKDTreeFactory>(*this);
        ar &numBins;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief How many bins use for the min-max approximation of loss function
     */
    size_t numBins;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Fast surface area heuristic KDTree factory default constructor
     * @param numBins How many bins the \f$H_*\f$ and \f$H^*\f$ histograms must
     *  use
     * @see SAHKDTreeFactory::SAHKDTreeFactory
     * @see FastSAHKDTreeFactory
     */
    FastSAHKDTreeFactory(
        size_t const numBins=32,
        size_t const lossNodes=21,
        double const ci=1,
        double const cl=1,
        double const co=1
    ) :
        SAHKDTreeFactory(lossNodes, ci, cl, co),
        numBins(numBins)
    {}
    virtual ~FastSAHKDTreeFactory () = default;

    // ***  SAH UTILS  *** //
    // ******************* //
    /**
     * @brief Compute an approximation of the loss function for the splitting
     *  hyperplane using a min-max discrete method
     *
     * @see SAHKDTreeFactory::splitLoss
     * @see FastSAHKDTreeFactory
     */
    double splitLoss(
        vector<Primitive *> const &primitives,
        int const splitAxis,
        double const splitPos,
        double const r
    ) const override;

    /**
     * @brief Find the best split position using an min-max like approximation
     *  for the loss function of Surface Area Heuristic (SAH)
     * @return Approximate loss of best split position. The position itself is
     *  already stored in given node
     * @see SAHKDTreeFactory::findSplitPositionBySAH
     * @see SAHKDTreeFactory::defineSplit
     */
    double findSplitPositionBySAH(
        KDTreeNode *node,
        vector<Primitive *> &primitives
    ) const override;
};