#pragma once

#include <SimpleKDTreeFactory.h>

class MultiThreadSAHKDTreeFactory;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for k-dimensional trees with surface
 *  area heuristic (SAH)
 *
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
 *  the cost of the tree where \f$R\f$ is the root node as follows, where
 *  \f$N_o(l)\f$ is the number of objects in the \f$l\f$-th leaf:
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
 * Now, let \f$r\f$ be the normalized position of the splitting hyperplane
 *  for node \f$N\f$ so \f$r=0\f$ is the lower limit, \f$r=1\f$ is the upper
 *  limit and \f$r=\frac{1}{2}\f$ is the center. Moreover, let \f$L_r\f$ and
 *  \f$R_r\f$ be the left and right parts for the \f$r\f$ split position and
 *  \f$N_o(L_r)\f$ and \f$N_o(R_r)\f$ be the number of objects at the left and
 *  right splits respectively. In consequence, following loss function arises:
 *
 * \f[
 *  \mathcal{L}(r) = S_A(L_r)N_o(L_r) + S_A(R_r)N_o(R_r) - S_A(N)N_o(N)
 * \f]
 *
 * Alternatively, considering the term \f$-S_A(r)n\f$ is the amount of
 *  work saved by making the node an interior one (so the minus sign), it
 *  can be treated as a constant so for the sake of simplicity it would
 *  lead to:
 *
 * \f[
 *  \begin{array}{lll}
 *  \mathcal{L}(r)  &=& S_A(L_r)N_o(L_r) + S_A(R_r)N_o(R_r) \\
 *                  &=& rS_A(N)N_o(L_r) + (1-r)S_A(N)N_o(R_r) \\
 *                  &=& S_A(N) \left[rN_o(L_r) + (1-r)N_o(R_r)\right]
 *  \end{array}
 * \f]
 *
 * Differentiating with respect to \f$r\f$ leads to:
 * \f[
 *  \frac{d\mathcal{L}}{dr} =
 *      \left(2N_o(L_r) - N_o(N)\right)\frac{d}{dr}S_A(L_r) +
 *      \left[
 *          S_A(L_r) - S_A(R_r)
 *      \right]
 *      \frac{d}{dr}N_o(L_r)
 * \f]
 *
 * Although \f$N_o(L_r)\f$ is a discontinuous function, which implies
 *  \f$\frac{d}{dr}N_o(L_r)\f$ is not defined, it is known that is always
 *  nonnegative which is enough to define a valid minimization criteria. In
 *  consequence, it is possible to analyze different scenarios. First,
 *  consider the case where the median lies somewhere satisfying
 *  \f$r < \frac{1}{2}\f$. Thus, \f$\frac{d}{dr}\mathcal{L}(r) < 0\f$ at
 *  the left side because
 *  \f$N_o(L_r) < \frac{n}{2}\f$ and \f$S_A(L_r) < S_A(R_r)\f$. On the other
 *  hand, \f$\frac{d}{dr}\mathcal{L}(r) > 0\f$ at the right side because
 *  \f$N_o(L_r) < \frac{n}{2}\f$ and \f$S_A(L_r) > S_A(R_r)\f$. So the minimum
 *  must occur between the object median and the spatial median if the
 *  object median is to the left of the spatial median. It is easy to see
 *  that an analogous argument applies for the case where the object median
 *  is to the right of the spatial median. Then, the optimum split must
 *  lie between the object median and the spatial median (center).
 *
 * To clarify, the object median is understood as related to the splitting
 *  plane that places one half of the objects on each side of the plane.
 *  While the spatial median \f$\mu\f$ for a given KDTree node in
 *  \f$\mathbb{R}^{n}\f$ with \f$a = (a_1, \ldots, a_n)\f$ as minimum vertex
 *  and \f$b = (b_1, \ldots, b_n)\f$ as maximum vertex is:
 *
 * \f[
 *  \mu = \frac{a+b}{2}
 *      = \left(\frac{a_1+b_1}{2}, \ldots, \frac{a_n+b_n}{2}\right)
 * \f]
 *
 * <i>For a more detailed explanation refer to "Heuristics for ray tracing
 *  using space subdivision" by J. David MacDonald and Kellogg S. Booth.
 *  </i>
 *
 * @see SimpleKDTreeFactory
 * @see AxisSAHKDTreeFactory
 */
class SAHKDTreeFactory : public SimpleKDTreeFactory{
    // ***  FRIENDS  *** //
    // ***************** //
    friend class MultiThreadSAHKDTreeFactory;

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
        ar &ci &cl &co;
        ar &lossNodes;
        ar &cacheI &cacheL &cacheO &cacheT;
        ar &cacheRoot;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief How many loss nodes must be computed when optimizing the
     *  loss function \f$\mathcal{L}_2\f$ to determine the best split
     *  position for a given KDTree node
     * @see SAHKDTreeFactory::splitLoss
     */
    size_t lossNodes;
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
     * @brief Cache pointer to root node of current KDTree being built
     */
    KDTreeNode *cacheRoot = nullptr;
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
    /**
     * @brief Function to lock the ILOT cache on a unique way (no thread but
     *  locker one must be able to use it). By default it is a void function,
     *  it must be overridden to provide concurrency handling.
     * @see MultiThreadSAHKDTreeFactory
     * @see SAHKDTreeFactory::_unlockILOT
     */
    std::function<void(void)> _lockILOT;
    /**
     * @brief Function to unlock the ILOT cache. It is the counterpart of the
     *  _lockILOT function
     * @see MultiThreadSAHKDTreeFactory
     * @see SAHKDTreeFactory::_lockILOT
     */
    std::function<void(void)> _unlockILOT;

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
        size_t const lossNodes=21,
        double const ci=1,
        double const cl=1,
        double const co=1
    );
    virtual ~SAHKDTreeFactory() = default;

    // ***  BUILDING METHODS  *** //
    // ************************** //
    /**
     * @brief Define the split axis and position for current node
     *
     * To illustrate the define split method let \f$a\f$ and \f$b\f$ be the
     *  minimum and maximum vertices of given node. Let also \f$\mu\f$ be the
     *  geometric center or spatial median and \f$M_e\f$ be the object median.
     *  For the sake of simplicity, lets assume \f$\mu < M_e\f$ so the
     *  iterative method will start at \f$\mu\f$ and end at \f$M_e\f$. If it
     *  was the other way, then the iterative method would start at \f$M_e\f$
     *  and end at \f$\mu\f$.
     *  Now, if \f$n\f$ is the number of loss nodes, \f$\mathcal{L}_2\f$ is
     *  the loss function and \f$r = \frac{\phi-a}{b-a} \in [0, 1]\f$ is the
     *  normalized position of split position \f$\phi\f$. Then, the iterative
     *  method can be defined as:
     *
     * \f[
     *  \varphi(t) = \mu + t \frac{M_e-\mu}{n-1} \\
     *  \left\{\begin{array}{lll}
     *  \phi_1 &=& \mu \\
     *  \phi_{t>1} &=& \left\{\begin{array}{lll}
     *      \varphi(t)  &,&
     *          \mathcal{L}_2\left(\varphi(t)\right) <
     *          \mathcal{L}_2\left(\phi_{t-1}\right) \\
     *      \phi_{t-1}  &,&
     *          \mathcal{L}_2\left(\varphi(t)\right) \geq
     *          \mathcal{L}_2\left(\phi_{t-1}\right)
     *  \end{array}\right.
     *  \end{array}\right.
     * \f]
     *
     * Finally, \f$\phi_n\f$ is the best found split position.
     *
     * Notice that the median is constrained so \f$M_e \in [a, b]\f$. Thus,
     *  in case there are enough objects lying outside node boundaries causing
     *  the median to be also outside, it will be truncated.
     *
     * @see SimpleKDTreeFactory::defineSplit
     * @see SAHKDTreeFactory::computeKDTreeStats
     * @see SAHKDTreeFactory::buildChildrenNodes
     * @see SAHKDTreeFactory::splitLoss
     * @see SAHKDTreeFactory::lossNodes
     * @see SAHKDTreeFactory::findSplitPositionBySAH
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
     * If root node, then by default assume it is a leaf node containing
     *  all primitives:
     * \f[
     *  t_0 : \mathrm{ILOT} = C_T = \frac{1}{S_A(R)} \left[
     *      C_lS_A(R) + C_oS_A(l)N_o(R)
     *  \right]
     * \f]
     *
     * After this, speculate its cost if it is make an interior node with
     *  its primitives being splitted into 2 leaf nodes:
     * \f[
     *  C_{Sr} = \frac{1}{S_A(R)} \left[
     *      C_i S_A(R) +
     *      C_l \sum_{l=1}^{2} {S_A(l)} +
     *      C_o \sum_{l=1}^{2} {S_A(l)N_o(l)}
     *  \right]
     * \f]
     *
     * Now if \f$C_{Sr} \geq C_T\f$ at \f$t_0\f$ then the process is stopped
     *  and all primitives remain in the root node. Otherwise, \f$t_1\f$
     *  happend so:
     * \f[
     *  t_1 : \mathrm{ILOT} = C_T = C_{Sr}
     * \f]
     *
     * After the initial case, a similar process is recursively applied to each
     *  new node until \f$C_{Si} \geq C_{T}\f$. It is, until the speculative
     *  cost is found to be greater or equal than current cost. This new
     *  speculative cost for non-root interior nodes is computed as follows,
     *  where \f$N\f$ is current leaf node which might be splitted depending on
     *  analysis and \f$L_r\f$ and \f$R_r\f$ are its left and right splits
     *  respectively:
     * \f[
     * \left\{\begin{array}{lll}
     *  k_1 &=& C_i S_A(N) \\
     *  k_2 &=&  C_l \left(S_A(L_r) + S_A(R_r)\right) - C_l S_A(N) \\
     *  k_3 &=& C_o \left(S_A(L_r)N_o(L_r) + S_A(R_r)N_o(R_r)\right)
     *      - C_o S_A(N)N_o(N) \\
     *  C_{Si} &=& \frac{1}{S_A(R)} \left[
     *      C_i \sum_{i=1}^{N_i} S_A(i) + k_1 +
     *      C_l \sum_{l=1}^{N_l} S_A(l) + k_2 +
     *      C_o \sum_{l=1}^{N_l} S_A(l)N_o(l) + k_3
     *  \right]
     * \end{array}\right.
     * \f]
     *
     * Thus, iterations \f$x>1\f$ that will only happen when \f$C_{Si} < C_T\f$
     *  can be defined as:
     * \f[
     *  t_{x>1} : \mathrm{ILOT} = C_T = C_{Si}
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
        vector<Primitive *> &leftPrimitives,
        vector<Primitive *> &rightPrimitives
    ) override;

protected:
    // ***  SAH UTILS  *** //
    // ******************* //
    /**
     * @brief Compute the loss function for the splitting hyperplane
     *
     * The loss function by default is:
     *
     * \f[
     *  \begin{array}{lll}
     *  \mathcal{L}(r)  &=& S_A(L_r)N_o(L_r) + S_A(R_r)N_o(R_r) \\
     *                  &=& S_A(N) [rN_o(L_r) + (1-r)N_o(R_r)]
     *  \end{array}
     * \f]
     *
     *
     * However, as \f$S_A(N)\f$ is a constant for the same node, it is
     *  computationally cheaper to compute an alternative version:
     *
     * \f[
     *  \mathcal{L_2}(r) = rN_o(L_r) + (1-r)N_o(R_r)
     * \f]
     *
     * @param primitives Vector of primitives involved in the split
     * @param splitAxis Axis at which split will be done
     * @param splitPos Position of the hyperplane in the split axis
     * @param r The ratio or normalized split position in \f$[0, 1]\f$
     * @return Value obtained after evaluating loss function
     * @see SAHKDTreeFactory::defineSplit
     * @see SAHKDTreeFactory::lossNodes
     */
    virtual double splitLoss(
        vector<Primitive *> const &primitives,
        int const splitAxis,
        double const splitPos,
        double const r
    ) const ;

    /**
     * @brief Find the best split position using Surface Area Heuristic (SAH)
     *  as described in SAHKDTreeFactory::defineSplit
     * @return Loss of best split position. The position itself is already
     *  stored in given node
     * @see SAHKDTreeFactory::defineSplit
     */
    virtual double findSplitPositionBySAH(
        KDTreeNode *node,
        vector<Primitive *> &primitives
    ) const;

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
    /**
     * @brief Compute ILOT corresponding to internalization (make interior)
     *  of given node and its corresponding left and right splits
     * @param[in] hi To store new interior heuristic cost
     * @param[in] hl To store new leaves heuristic cost
     * @param[in] ho To store new object heuristic cost
     * @param[in] ht To store new tree heuristic cost
     * @param node Node to internalize (make interior)
     * @param primitives Primitives on given node
     * @param leftPrimitives Primitives on left split
     * @param rightPrimitives Primitives on right split
     */
    virtual void internalizeILOT(
        double &hi,
        double &hl,
        double &ho,
        double &ht,
        KDTreeNode *node,
        vector<Primitive *> const &primitives,
        vector<Primitive *> const &leftPrimitives,
        vector<Primitive *> const &rightPrimitives
    );

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
     * @see SAHKDTreeFactory::fromILOCache
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
    /**
     * @brief Set references from ILOT cache but only for ILO components
     * @see SAHKDTreeFactory::cacheI
     * @see SAHKDTreeFactory::cacheL
     * @see SAHKDTreeFactory::cacheO
     * @see SAHKDTreeFactory::fromILOTCache
     */
    virtual inline void fromILOCache(
        double &I,
        double &L,
        double &O
    ) const {
        I = cacheI;
        L = cacheL;
        O = cacheO;
    }
    /**
     * @brief Obtain the T component of ILOT cache
     * @return T component of ILOT cache
     */
    virtual inline double getCacheT() const {return cacheT;}
    /**
     * @brief Initialize the ILOT cache from given root node
     * @param root Root node to initialize ILOT cache from
     * @param primitives Primitives contained in root node
     */
    virtual void initILOT(
        KDTreeNode *root,
        vector<Primitive *> const &primitives
    );
    /**
     * @brief Set the cached root node
     * @param root The new root node to be cached
     * @see SAHKDTreeFactory::cacheRoot
     */
    virtual inline void setCacheRoot(KDTreeNode *root) {cacheRoot = root;}

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
