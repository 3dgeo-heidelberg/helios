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

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Cost-weight for traversing interior nodes
     * @see SAHKDTreeFactory::defineSplit
     */
    double ci;
    /**
     * @brief Cost-weight for traversing leaf nodes
     * @see SAHKDTreeFactory::defineSplit
     */
    double cl;
    /**
     * @brief Cost-weight for testing an object for intersection
     * @see SAHKDTreeFactory::defineSplit
     */
    double co;

    // ***  CACHE ATTRIBUTES  *** //
    // ************************** //
    /**
     * @brief Cache last valid interior cost
     *
     * \f[
     *  C_i \sum_{i=1}^{N_i}{S_A(i)}
     * \f]
     * @see SAHKDTreeFactory::toILOTCache
     * @see SAHKDTreeFactory::fromILOTCache
     */
    double cacheI;
    /**
     * @brief Cache last valid leaves cost
     *
     * \f[
     *  C_l \sum_{l=1}^{N_l}{S_A(l)}
     * \f]
     * @see SAHKDTreeFactory::toILOTCache
     * @see SAHKDTreeFactory::fromILOTCache
     */
    double cacheL;
    /**
     * @brief Cache last valid object cost
     *
     * \f[
     *  C_o \sum_{l=1}^{N_l}{S_A(l) N_o(l)}
     * \f]
     * @see SAHKDTreeFactory::toILOTCache
     * @see SAHKDTreeFactory::fromILOTCache
     */
    double cacheO;
    /**
     * @brief Cache last valid tree cost
     *
     * \f[
     *  C_T = \frac{
     *      C_i \sum_{i=1}^{N_i}{S_A(i)} +
     *      C_l \sum_{l=1}^{N_l}{S_A(l)} +
     *      C_o \sum_{l=1}^{N_l}{S_A(l) N_o(l)}
     *  }
     *  {S_A(R)}
     * \f]
     * @see SAHKDTreeFactory::toILOTCache
     * @see SAHKDTreeFactory::fromILOTCache
     */
    double cacheT;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Surface area heuristic KDTree factory default constructor
     * @see SAHKDTreeFactory::ci
     * @see SAHKDTreeFactory::cl
     * @see SAHKDTreeFactory::co
     */
    SAHKDTreeFactory (
        double const ci=1,
        double const cl=2,
        double const co=2
    ) :
        SimpleKDTreeFactory(),
        ci(ci),
        cl(cl),
        co(co)
    {}
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
     *      C_o \sum_{l=1}^{N_l}{S_A(l) N_o(l)}
     *  }
     *  {S_A(R)}
     * \f]
     *
     * Alternatively, let \f$S_l(o)\f$ be the set of leaves in which object
     *  \f$o\f$ resides so the cost of the tree can be also defined as:
     *
     * \f[
     *  C'_T = \frac{
     *      C_i \sum_{i=1}^{N_i}{S_A(i)} +
     *      C_l \sum_{l=1}^{N_l}{S_A(l)} +
     *      C_o \sum_{o=1}^{N_o}{S_A[S_l(o)]}
     *  }
     *  {S_A(R)}
     * \f]
     *
     * The main difference between \f$C_T\f$ and \f$C'_T\f$ would be that the
     *  second one fits better the case where a ray is not going to intersect
     *  multiple times the same object.
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
     * To clarify even more, consider the split axis is the \f$j\f$-th axis.
     *  Then, the spatial median would be:
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
     * @see SAHKDTreeFactory::computeKDTreeStats
     * @see SAHKDTreeFactory::buildChildrenNodes
     */
    void defineSplit(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> &primitives,
        int const depth
    ) const override;

    /**
     * @brief Compute the simple stats of the KDTree but also its cost based
     *  on surface area heuristic defined in defineSplit function.
     *
     * Current tree cost is computed as \f$C_T\f$ not as \f$C'_T\f$
     *
     * @see SimpleKDTreeFactory::computeKDTreeStats
     * @see SAHKDTreeFactory::defineSplit
     * @see SAHKDTreeFactory::buildChildrenNodes
     */
    void computeKDTreeStats(KDTreeNodeRoot *root) const override;

    /**
     * @brief Build children nodes using \f$C_T\f$ heuristic to handle KDTree
     *  in-depth partitioning
     *
     * If root node, then:
     * \f[
     *  t_0 : ILOT = C_T = \frac{1}{S_A(R)} \left[
     *      C_lS_A(l) + C_oS_A(l)N_o(l)
     *  \right]
     * \f]
     *
     * @see SimpleKDTreeFactory::buildChildrenNodes
     * @see SAHKDTreeFactory::defineSplit
     * @see SAHKDTreeFactory::computeKDTreeStats
     */
    void buildChildrenNodes(
        KDTreeNode *node,
        KDTreeNode *parent,
        vector<Primitive *> const &primitives,
        int const depth,
        vector<Primitive *> const &leftPrimitives,
        vector<Primitive *> const &rightPrimitives
    ) override;

protected:
    // ***  SAH UTILS  *** //
    // ******************* //
    /**
     * @brief Compute the loss function for the splitting hyperplane
     *
     * \f[
     *  \mathcal{L}(b) = S_A(L_b)L(b) + S_A(R_b)R(b)
     * \f]
     *
     * @param primitives Vector of primitives involved in the split
     * @param splitAxis Axis at which split will be done
     * @param splitPos Position of the hyperplane in the split axis
     * @param surfaceArea Surface area of node being splitted
     * @param b The normalized split position in \f$[0, 1]\f$
     * @return Value obtained after evaluating loss function
     */
    virtual double splitLoss(
        vector<Primitive *> const &primitives,
        int const splitAxis,
        double const splitPos,
        double const surfaceArea,
        double const b
    ) const ;

    /**
     * @brief Compute the \f$C_T\f$ heuristic preserving partials result of
     *  interest
     *
     * Costs and previous values are taken from factory and ILOT cache
     *  respectively
     *
     * @param[out] hi Where the partial interior cost will be stored
     * @param[out] hl Where the partial leaves cost will be stored
     * @param[out] ho Where the partial object cost will be stored
     * @param[out] ht Where the total tree cost will be stored
     * @param[in] surfaceAreaRoot Surface area of root node
     * @param[in] surfaceAreaInterior Surface area of interior node
     * @param[in] surfaceAreaLeaf Surface area of object/leaf node
     * @param[in] primitives Primitives defining the object/leaf cluster
     * @return \f$C_T\f$ as heuristic ILOT
     */
    virtual double heuristicILOT(
        double &hi,
        double &hl,
        double &ho,
        double &ht,
        double const surfaceAreaRoot,
        double const surfaceAreaInterior,
        double const surfaceAreaLeaf,
        vector<Primitive *> const &primitives
    ) const ;

    /**
     * @brief Compute the cumulative of \f$C_T\f$ heuristic ILOT.
     *
     * <b><span style="color: red">WARNING</span></b> given reference to
     *  current total tree cost is also updated, not only returned. Thus, this
     *  is a full ILOT write function
     *
     * @param hi Current partial interior cost to be updated
     * @param hl Current partial leaves cost to be updated
     * @param ho Current partial object cost to be updated
     * @param ht Current total tree cost to be updated
     * @param[in] _hi Adding magnitude to update partial interior cost
     * @param[in] _hl Adding magnitude to update partial leaves cost
     * @param[in] _ho Adding magnitude to update partial objeccts cost
     * @param[in] saRoot Surface area of root node \f$S_A(R)\f$ defining
     *  cumulative ILOT
     * @return Updated total tree cost
     */
    virtual double cumulativeILOT(
        double &hi,
        double &hl,
        double &ho,
        double &ht,
        double const _hi,
        double const _hl,
        double const _ho,
        double const saRoot
    ) const ;

    // ***  CACHE UTILS  *** //
    // ********************* //
    /**
     * @brief Set ILOT cache from given values
     * @see SAHKDTreeFactory::cacheI
     * @see SAHKDTreeFactory::cacheL
     * @see SAHKDTreeFactory::cacheO
     * @see SAHKDTreeFactory::cacheT
     * @see SAHKDTreeFactory::fromILOTCache
     */
    virtual inline void toILOTCache(
        double const I,
        double const L,
        double const O,
        double const T
    ){
        cacheI = I;
        cacheL = L;
        cacheO = O;
        cacheT = T;
    }
    /**
     * @brief Set references from ILOT cache
     * @see SAHKDTreeFactory::cacheI
     * @see SAHKDTreeFactory::cacheL
     * @see SAHKDTreeFactory::cacheO
     * @see SAHKDTreeFactory::cacheT
     * @see SAHKDTreeFactory::toILOTCache
     */
    virtual inline void fromILOTCache(
        double &I,
        double &L,
        double &O,
        double &T
    ) const {
        I = cacheI;
        L = cacheL;
        O = cacheO;
        T = cacheT;
    }

public:
    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the cost-weight of interior nodes
     * @return Cost-weight of interior nodes
     * @see SAHKDTreeFactory::ci
     */
    virtual inline double getInteriorCost() const {return ci;}
    /**
     * @brief Set the cost-weight of interior nodes
     * @param ci New cost-weight for interior nodes
     * @see SAHKDTreeFactory::ci
     */
    virtual inline void setInteriorCost(double const ci) {this->ci = ci;}
    /**
     * @brief Obtain the cost-weight for leaf nodes
     * @return Cost-weight of leaf nodes
     * @see SAKHDTreeFactory::cl
     */
    virtual inline double getLeafCost() const {return cl;}
    /**
     * @brief Set the cost-weight for leaf nodes
     * @param cl New cost-weight for leaf nodes
     * @see SAHKDTreeFactory::cl
     */
    virtual inline void setLeafCost(double const cl) {this->cl = cl;}
    /**
     * @brief Obtain the cost-weight of testing an object for intersection
     * @return Cost-weight of testing an object for intersection
     * @see SAHKDTreeFactory::co
     */
    virtual inline double getObjectCost() const {return co;}
    /**
     * @brief Set the cost-weight of testing an object for intersection
     * @param co New cost-weight of testing an object for intersection
     * @see SAHKDTreeFactory::co
     */
    virtual inline void setObjectCost(double const co) {this->co = co;}
};
