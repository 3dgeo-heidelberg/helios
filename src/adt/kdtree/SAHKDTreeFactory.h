#pragma once

#include <SimpleKDTreeFactory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for k-dimensional trees with surface
 *  area heuristic (SAH)
 *
 * @see SAHKDTreeFactory
 */
class SAHKDTreeFactory : public SimpleKDTreeFactory{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a surface area heuristic KDTree factory to a stream
     *  of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the surface area heuristic KDTree
     *  factory
     */
    template <typename Archive>
    void serialize(Archive &ar, unsigned int const version){
        boost::serialization::void_cast_register<
            SAHKDTreeFactory,
            SimpleKDTreeFactory
        >();
        ar &boost::serialization::base_object<SimpleKDTreeFactory>(*this);
    }

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Surface area heuristic KDTree factory default constructor
     */
    SAHKDTreeFactory () = default;
    virtual ~SAHKDTreeFactory() = default;

    // ***  BUILDING METHODS  *** //
    // ************************** //
    /**
     * @brief Define the split axis and position for current node
     *
     * The surface area heuristic KDTree factory defines the split point from
     *  the fact that an optimal performance point must be between the median
     *  of the distribution and the geometric center.
     *
     * Let \f$C_i\f$, \f$C_l\f$ and \f$C_o\f$ be the cost-weight for
     *  traversing interior nodes, traversing leaf nodes and testing an object
     *  for intersection respectively. Also, let \f$N_i\f$, \f$N_l\f$ and
     *  \f$N_o\f$ number of interior nodes, number of leaf nodes and number
     *  of objects. Considering \f$S_A(i)\f$ the surface area of the
     *  i-th interior node, \f$S_A(l)\f$ the surface area of the l-th leaf
     *  node and generically \f$S_A(x)\f$ the surface area of the \f$x\f$
     *  object or the \f$x\f$ set of objects. It is now possible to define
     *  the cost of the tree where \f$R\f$ is the root node as:
     *
     * \f[
     *  C_T = \frac{
     *      C_i \sum_{i=1}^{N_i}{S_A(i)} +
     *      C_l \sum_{l=1}^{N_l}{S_A(l)} +
     *      C_o \sum_{l=1}^{N_l}{S_A(l)N_o(l)}
     *  }
     *  {S_A(R)}
     * \f]
     *
     * Alternatively, let \f$S_l(o)\f$ be the set of leaves in which object
     *  \f$o\f$ resides so the cost of the tree can be also defined as:
     *
     * \f[
     *  C_T = \frac{
     *      C_i \sum_{i=1}^{N_i}{S_A(i)} +
     *      C_l \sum_{l=1}^{N_l}{S_A(l)} +
     *      C_o \sum_{l=1}^{N_l}{S_A[S_l(o)]}
     *  }
     *  {S_A(R)}
     * \f]
     *
     * Now, let \f$b\f$ be the normalized position of the splitting hyperplane
     *  so \f$b=0\f$ is the lower limit, \f$b=1\f$ is the upper limit and
     *  \f$b=\frac{1}{2}\f$ is the median. Moreover, let \f$L(b)\f$ and
     *  \f$R(b)\f$ be the number of objects at the left and right splits
     *  respectively and \f$L_b\f$ and \f$R_b\f$ the  left and right parts
     *  for the \f$b\f$ split position. In consequence, following loss function
     *  arises:
     *
     * \f[
     *  \mathcal{L}(b) = S_A(L_b)L(b) + S_A(R_b)R(b) - S_A(b)n
     * \f]
     *
     * Alternatively, considering the term \f$-S_A(b)n\f$ is the amount of
     *  work saved by making the node an interior one (so the minus sign), it
     *  can be treated as a constant so for the sake of simplicity it would
     *  lead to:
     *
     * \f[
     *  \mathcal{L}(b) = S_A(L_b)L(b) + S_A(R_b)R(b)
     * \f]
     *
     * Differentiating with respect to \f$b\f$ leads to:
     * \f[
     *  \frac{d\mathcal{L}}{db} =
     *      \left(2L(b) - n\right)\frac{d}{db}S_A(L_b) +
     *      \left[
     *          S_A(L_b) - S_A(R_b)
     *      \right]
     *      \frac{d}{db}L(b)
     * \f]
     *
     * Although \f$L(b)\f$ is a discontinuous function, which implies
     *  \f$\frac{d}{db}L(b)\f$ is not defined, it is known that is always
     *  nonnegative which is enough to define a valid minimization criteria. In
     *  consequence, it is possible to analyze different scenarios. First,
     *  consider the case where the median lies somewhere satisfying
     *  \f$b < \frac{1}{2}\f$. Thus, \f$\frac{d}{db}\mathcal{L}(b) < 0\f$ at
     *  the left side because
     *  \f$L(b) < \frac{n}{2}\f$ and \f$S_A(L_b) < S_A(R_b)\f$. On the other
     *  hand, \f$\frac{d}{db}\mathcal{L}(b) > 0\f$ at the right side because
     *  \f$L(b) < \frac{n}{2}\f$ and \f$S_A(L_b) > S_A(R_b)\f$. So the minimum
     *  must occur between the object median and the spatial median if the
     *  object median is to the left of the spatial median. It is easy to see
     *  that an analogous argument applies for the case where the object median
     *  is to the right of the spatial median. Then, the optimum split must
     *  lie between the object median and the spatial median.
     *
     * To clarify, the object median is understood as related to the splitting
     *  plane that places one half of the objects on each side of the plane.
     *  While the spatial median \f$\phi\f$ for a given set of \f$m\f$ vertices
     *  is defined as:
     *
     * \f[
     *  \mathrm{argmin}_{\phi} =
     *      \sum_{i=1}^{m} \left\Vert{p_i - \phi}\right\Vert_2 =
     *      \sum_{i=1}^{m} \sqrt{\left\langle
     *          {p_i-\phi, p_i-\phi}
     *      \right\rangle}
     * \f]
     *
     * To clarify, consider the split axis is the \f$j\f$-th axis. Then, the
     *  spatial median would be:
     * \f[
     *  \mathrm{argmin}_{\phi_j} =
     *      \sum_{i=1}^{m}{\left|p_{ij}-\phi_j\right|}
     * \f]
     *
     *
     * <i>For a more detailed explanation refer to "Heuristics for ray tracing
     *  using space subdivision" by J. David MacDonald and Kellogg S. Booth.
     *  </i>
     *
     * @see SimpleKDTreeFactory::defineSplit
     */
    void defineSplit(
        vector<Primitive *> &primitives,
        int const depth,
        int &splitAxis,
        double &splitPos
    ) const override;
};
