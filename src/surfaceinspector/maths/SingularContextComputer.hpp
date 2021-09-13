#ifndef _SURFACEINSPECTOR_MATHS_SINGULARCONTEXTCOMPUTER_HPP_
#define _SURFACEINSPECTOR_MATHS_SINGULARCONTEXTCOMPUTER_HPP_

#include <vector>
#include <armadillo>

#include <util/Object.hpp>
#include <maths/SingularContextDescriptors.hpp>

using std::vector;
using arma::Mat;

using SurfaceInspector::util::Object;
using SurfaceInspector::maths::SingularContextDescriptors;

namespace SurfaceInspector { namespace maths{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Handle singular context descriptors computation
 *
 * The singular context descriptors algorithm can be described as follows,
 *  for any input matrix \f$P_{m \times n}\f$ where each i-th row is a point
 *  \f$p_i = (x_{i1}, \ldots, x_{in})\f$ in \f$\mathbb{R}^{n}\f$.
 *
 * <b>1. Normalize</b><br/>
 *  Let \f$\vec{x}_j\f$ be the vector containing the j-th component of each
 *      point in the matrix \f$P\f$. Thus,
 *      \f$\vec{x}_j = (x_{1j}, \ldots, x_{mj})\f$ is a column vector from
 *      \f$P\f$.<br/>
 *  Now, each value in \f$P\f$ can be normalized as follows:
 *  \f[
 *      \hat{x}_{ij} = \frac{x_{ij} - \min\left({\vec{x}_j}\right)}
 *      {\max\left({\vec{x}_j}\right) - \min\left(\vec{x}_j\right)}
 *  \f]
 *
 *  <b>NOTICE</b> here exposed method is the default normalization strategy.
 *  Other normalization methods could be used.
 *
 * <hr/><b>2. Center</b><br/>
 * Let \f$\mu_j\f$ be the mean of the j-th component for each normalized
 *  point. It is:
 *  \f[
 *      \mu_j = \frac{1}{m} \sum_{i=1}^{m}{\hat{x}_{ij}}
 *  \f]
 * Therefore, for each normalized \f${\hat{x}_{ij}}\f$ its centered version
 *  can be defined as \f$\overline{x}_{ij} = \hat{x}_{ij} - \mu_j\f$.
 *
 * <b>NOTICE</b> here exposed method is the default centering strategy. Other
 *  centering methods could be used.
 *
 * <hr/><b>3. Factorize</b><br/>
 * After applying previous transformations, the matrix \f${X}\f$ arises:
 * \f[
 *  X_{m \times n} = \left[\begin{array}{lll}
 *      \overline{x}_{11} & \ldots & \overline{x}_{1n} \\
 *      \vdots & \ddots & \vdots \\
 *      \overline{x}_{m1} & \ldots & \overline{x}_{mn}
 *  \end{array}\right]
 * \f]
 *
 * This matrix can be factorized with singular value decomposition so:
 * \f[
 *  X_{m \times n } = U_{m \times r} \Sigma_{r \times r} V^{T}_{r \times n}
 * \f]
 *
 * In consequence, singular values are obtained as
 *  \f$\mathrm{diag}\left(\Sigma\right) = \vec{\sigma}\f$ while singular
 *  vectors are the rows from matrix \f$V^{T}\f$.
 *
 * Currently, armadillo library is being used to compute the singular values
 *  decomposition. This library returns \f$V\f$ instead of \f$V^{T}\f$. So,
 *  if the singular vectors come from rows of \f$V^{T}\f$, when using armadillo
 *  they come from columns of \f$V\f$.
 *
 * <hr/><b>4. Singular context descriptors</b><br/>
 * Let \f$v^{-}\f$ and \f$v^{+}\f$ be the singular vectors associated with
 *  minimum and maximum singular values, respectively.
 * Also, let the functions \f$\alpha_a(\vec{v})\f$ and
 *  \f$\beta_b(\vec{v})\f$ be as described in this class.
 *
 * Thus, sets \f$\mathcal{S}^{-}\f$ and \f$\mathcal{S}^{+}\f$ will be:
 * \f[
 *  \left.\begin{array}{l}
 *      \mathcal{S}^{-} = \left\{
 *          (j, v_j^-) : j \in \alpha_a(v^-) \lor j \in \beta_b(v^-)
 *      \right\} \\
 *      \mathcal{S}^{+} = \left\{
 *          (j, v_j^+) : j \in \alpha_a(v^+) \lor j \in \beta_b(v^+)
 *      \right\}
 *  \end{array}\right.
 * \f]
 *
 * Now understand \f$\sigma^-\f$ and \f$sigma^+\f$ as the minimum and maximum
 *  singular values respectively.
 *
 * Finally, the singular context descriptors of \f$X\f$ are:
 * \f[
 *  \mathrm{SCD}\left(X \right) = \left\{
 *      \sigma^-, \sigma^+, \mathcal{S}^-, \mathcal{S}^+
 *  \right\}
 * \f]
 *
 * @see SurfaceInspector::maths::SingularContextDescriptors
 * @see SingularContextComputer::alpha
 * @see SingularContextComputer::beta
 */
template <typename T>
class SingularContextComputer : public Object{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Flag which specifies if the singular context computer must
     *  normalize when computing descriptors (true) or not (false)
     */
    bool normalizeFlag;
    /**
     * @brief Flag which specifies if the singular context computer must
     *  center when computing descriptors (true) or not (false)
     */
    bool centerFlag;
    /**
     * @brief How many minimum values consider when computing the
     *  \f$\alpha_a(v^{-})\f$ function for the worst fitting vector
     */
    size_t aWorst;
    /**
     * @brief How many minimum values consider when computing the
     *  \f$\alpha_a(v^{+})\f$ function for the best fitting vector
     */
    size_t aBest;
    /**
     * @brief How many minimum values consider when computing the
     *  \f$\beta_b(v^{-})\f$ function for the worst fitting vector
     */
    size_t bWorst;
    /**
     * @brief How many maximum values consider when computing the
     *  \f$\beta_b(v^{+})\f$ function for the best fitting vector
     */
    size_t bBest;

    /**
     * @brief Specify if use absolute values of the singular vector components
     *  (true) or consider their signed version (false)
     */
    bool absoluteComponents = false;
    /**
     * @brief Function which normalizes the matrix if necessary.
     *  A default implementation is provided as the normalizeDefault function.
     *
     * Any valid normalization function that is assigned to this attribute
     *  will be used by the singular context computer
     *
     * @see SingularContextComputer::normalizeDefault
     */
    void (SingularContextComputer<T>::*normalize) (Mat<T> &M) const;
    /**
     * @brief Function which centers the matrix if necessary.
     *  A default implementation is provided as the centerDefault function.
     *
     * Any valid centering function that is assigned to this attribute
     *  will be used by the singular context computer
     *
     * @see SingularContextComputer::centerDefault
     */
    void (SingularContextComputer<T>::*center) (Mat<T> &M) const;

    /**
     * @brief The \f$\vec{a}\f$ vector must contain the start of normalization
     *  interval if the normalization strategy requires it.
     *
     * \f$a_i\f$ is the start of normalization interval for i-th component
     */
    vector<T> aNorm;
    /**
     * @brief The \f$\vec{b}\f$ vector must contain the end of normalization
     *  interval if the normalization strategy requires it.
     *
     * \f$b_i\f$ is the end of normalization interval for i-th component
     */
    vector<T> bNorm;
    /**
     * \f[
     *  \vec{\Delta} = \vec{b} - \vec{a}
     * \f]
     */
    vector<T> deltaNorm;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for singular context computer
     * @see SingularContextComputer::init
     */
    SingularContextComputer()
    {init(true, true, 1, 1, 1, 1);}
    /**
     * @brief Non default constructor for singular context computer
     * @see SingularContextComputer::init
     */
    SingularContextComputer(bool normalize, bool center)
    {init(normalize, center, 1, 1, 1, 1);}
    /**
     * @brief Non default constructor for singular context computer
     * @see SingularContextComputer::init
     */
    SingularContextComputer(
        size_t aWorst,
        size_t aBest,
        size_t bWorst,
        size_t bBest
    )
    {init(true, true, aWorst, aBest, bWorst, bBest);}
    /**
     * @brief Non default constructor for singular context computer
     * @see SingularContextComputer::init
     */
    SingularContextComputer(
        bool normalize,
        bool center,
        size_t aWorst,
        size_t aBest,
        size_t bWorst,
        size_t bBest
    )
    {init(normalize, center, aWorst, aBest, bWorst, bBest);}
    virtual ~SingularContextComputer() {};

    // ***  INITIALIZE  *** //
    // ******************** //
    /**
     * @brief Initialize the singular context computer
     * @see SingularContextComputer::normalize
     * @see SingularContextComputer::center
     * @see SingularContextComputer::aWorst
     * @see SingularContextComputer::aBest
     * @see SingularContextComputer::bWorst
     * @see SingularContextComputer::bBest
     */
    void init(
        bool normalize,
        bool center,
        size_t aWorst,
        size_t aBest,
        size_t bWorst,
        size_t bBest
    );

    // ***  MAIN FUNCTIONS  *** //
    // ************************ //
    /**
     * @brief Obtain singular context descriptors from given matrix
     *
     * <b>NOTICE</b> the describe function might modify given matrix
     *
     * @param M Matrix which singular context descriptors must be obtained
     * @return The singular context descriptors of given matrix
     * @see SingularContextDescriptors
     */
    SingularContextDescriptors<T> describe(Mat<T> &M) const;

    // ***  AUXILIAR FUNCTIONS  *** //
    // **************************** //
    /**
     * @brief Extract indices for \f$a\f$ minimum values of \f$\vec{v}\f$
     * @param v Vector to extract indices of \f$a\f$ minimum values from
     * @param a How many minimum values extract from vector \f$\vec{v}\f$
     * @return Indices of \f$a\f$ minimum values from \f$\vec{v}\f$
     */
    vector<size_t> alpha(arma::vec const &v, size_t const a) const;
    /**
     * @brief Extract indices for \f$b\f$ maximum values of \f$\vec{v}\f$
     * @param v Vector to extract indices of \f$b\f$ maximum values from
     * @param b How many maximum values extract from vector \f$\vec{v}\f$
     * @return Indices of \f$b\f$ maximum values from \f$\vec{v}\f$
     */
    vector<size_t> beta(arma::vec const &v, size_t const b) const;
    /**
     * @brief Extract components of given vector \f$\vec{v}\f$ corresponding to
     *  given indices
     * @param v Vector to extract components from
     * @param indices Indices of components to be extracted
     * @return Vector containing extracted components
     */
    vector<T> extractComponents(
        arma::vec const &v,
        vector<size_t> indices
    ) const;

    // ***  NORMALIZATION  *** //
    // *********************** //
    /**
     * @brief Configure normalization vectors from given ones
     * @param a The \f$\vec{a}\f$ vector for normalization start
     * @param b The \f$\vec{b}\f$ vector for normalization end
     * @see SingularContextComputer::aNorm
     * @see SingularContextComputer::bNorm
     * @see SingularContextComputer::deltaNorm
     */
    void configureNorm(vector<T> const &a, vector<T> const &b);
    /**
     * @brief Configure normalization vectors from given matrix.
     * @param X Matrix containing all values so X[i][j] is the j-th value
     *  of the i-th attribute. It is, X[i] is the vector containing all values
     *  for i-th attribute.
     */
    void configureNorm(vector<vector<T>> const &X);
    /**
     * @brief Normalize given matrix by default criteria
     * @param M Matrix to be normalized
     */
    void normalizeDefault(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 3 components are RGB
     *  so they must be inside \f$[0, 255]\f$ interval.
     *
     * \f[
     *  \forall j \in \{1, 2, 3\}, \hat{x_ij} = \frac{x}{255}
     * \f]
     *
     * @param M Matrix to be normalized
     */
    void normalizeRGB(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 3 components are HSO
     *  (Horizontality, Sum and Omnivariance).
     *
     * @param M Matrix to be normalized
     */
    void normalizeHSO(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 3 components are HLI
     *  (Horizontality, Linearity and Intensity).
     *
     * @param M Matrix to be normalized
     */
    void normalizeHLI(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are LP
     *  (Linearity and Planarity)
     *
     * @param M Matrix to be normalized
     */
    void normalizeLP(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are GdCd
     *  (Ground-distance and Ceiling-distance)
     *
     * @param M Matrix to be normalized
     */
    void normalizeGdCd(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 3 components are
     *  GdCdI (Ground-distance, Ceiling-distance, Intensity
     *
     * @param M Matrix to be normalized
     */
    void normalizeGdCdI(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are HV
     *  (Horizontality and Verticality)
     *
     * @param M Matrix to be normalized
     */
    void normalizeHV(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are SO
     *  (Sum and Omnivariance)
     *
     * @param M Matrix to be normalized
     */
    void normalizeSO(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are LC
     *  (Linearity and Curvature)
     *
     * @param M Matrix to be normalized
     */
    void normalizeLC(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are VC
     *  (Verticality and Curvature)
     *
     * @param M Matrix to be normalized
     */
    void normalizeVC(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are PC
     *  (Planarity and Curvature)
     *
     * @param M Matrix to be normalized
     */
    void normalizePC(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming each first 2 components are VR
     *  (Verticality and Roughness)
     *
     * @param M Matrix to be normalized
     */
    void normalizeVR(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming (X,Y,Z) components for a cone
     *
     * @param M Matrix to be normalized
     */
    void normalizeCone(Mat<T> &M) const;
    /**
     * @brief Normalize given matrix assuming (X,Y,Z) components for a
     *  hyperbolic paraboloid
     *
     * @param M Matrix to be normalized
     */
    void normalizeHyperbolicParaboloid(Mat<T> &M) const;

    // ***  CENTERING  *** //
    // ******************* //
    /**
     * @brief Center given matrix by default criteria
     * @param M Matrix to be centered
     */
    void centerDefault(Mat<T> &M) const;
};
}}

#include <maths/SingularContextComputer.tpp>

#endif
