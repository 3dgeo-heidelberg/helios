#pragma once

#include <armadillo>
#include <boost/serialization/serialization.hpp>

using namespace arma;

namespace rigidmotion{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Interface that must be implemented by any class which represents
 *  a specific rigid motions
 *
 * A rigid motion is an affine application which preserves the distance between
 *  points. It is both, an endomorphism and an isometry. Any rigid motion in an
 *  affine euclidean space can be expressed as a composition of a
 *  transformation which preserves the origin with a translation.
 * Thus \f$f\f$ is a rigid motion if it satisfies:
 *
 * \f[
 *  \left\{\begin{array}{lll}
 *      f & : &\mathcal{A} \rightarrow \mathcal{A} \\
 *      \left\Vert{\overrightarrow{AB}}\right\Vert & = &
 *          \left\Vert{\overrightarrow{f(A)f(B)}}\right\Vert
 *  \end{array}\right.
 * \f]
 *
 * Any rigid motion can be expressed as an affine application \f$Y = C + AX\f$
 *  and also in block matrix notation as follows:
 *
 * \f[
 *  \left[\begin{array}{c|c}
 *     1 & 0 \\ \hline
 *     C & A
 *  \end{array}\right] =
 *  \left[\begin{array}{c|c}
 *      1 & 0 \\ \hline
 *      C & I
 *  \end{array}\right]
 *  \left[\begin{array}{c|c}
 *      1 & 0 \\ \hline
 *      0 & A
 *  \end{array}\right]
 * \f]
 */
class RigidMotion{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a rigid motion to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the rigid motion
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        ar &C;
        ar &A;
    }
protected:
    // ***  CONSTANTS  *** //
    // ******************* //
    /**
     * @brief Decimal precision tolerance
     */
    static double const eps;

    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The column vector representing the translation
     */
    colvec C;
    /**
     * @brief The matrix representing the fixed origin transformation
     */
    arma::mat A;

public:
    // ***  TYPE ENUMS  *** //
    // ******************** //
    /**
     * @brief Rigid motions supertypes. It is, the classification of a rigid
     *  motion attending to its fixed origin transformation matrix \f$A\f$
     *  only.
     *
     * For instance, a reflection and a glide reflection would be different
     *  types which share the same supertype (reflection).
     *
     * The supertype is determined attending to
     *  \f$\mathrm{rank}\left(I-A\right)\f$
     *
     * <b>NOTICE</b> both identity and translation transformations are
     *  considered as base supertype, since both of them have
     *  \f$\mathrm{rank}\left(I-A\right) = 0\f$
     *
     * @see rigidmotion::RigidMotion::findSuperType
     */
    enum SuperType{
        R2_BASE, R2_REFLECTION, R2_ROTATION,
        R3_BASE, R3_REFLECTION, R3_ROTATION, R3_ROTATIONAL_SYMMETRY
    };
    /**
     * @brief Rigid motions types. It is, the classification of a rigid
     *  motion attending to its associated isometry and fixed points.
     *
     * The type is fully determined attending to:
     * \f[
     *  \left\{\begin{array}{l}
     *   \mathrm{rank}\left(I-A\right) \\
     *   \mathrm{rank}\left(I-A | C\right)
     *  \end{array}\right.
     * \f]
     *
     * @see rigidmotion::RigidMotion::findType
     */
    enum Type{
        IDENTITY_R2, TRANSLATION_R2, REFLECTION_R2, GLIDE_REFLECTION_R2,
        ROTATION_R2,
        IDENTITY_R3, TRANSLATION_R3, REFLECTION_R3, GLIDE_REFLECTION_R3,
        ROTATION_R3, HELICAL_R3, ROTATIONAL_SYMMETRY_R3
    };
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief RigidMotion default constructor.
     * Notice rigid motions should be instantiated through corresponding
     * factories.
     * @see rigidmotion::RigidMotionFactory
     */
    RigidMotion() = default;
    /**
     * @brief Build a rigid motion with given translation vector and fixed
     *  origin transformation matrix
     * @param C Translation vector
     * @param A Fixed origin transformation matrix
     * @see rigidmotion::RigidMotion::C
     * @see rigidmotion::RigidMotion::A
     */
    RigidMotion(colvec const &C, arma::mat const &A) : C(C), A(A) {};
    virtual ~RigidMotion() = default;

    // ***  RIGID MOTION METHODS  *** //
    // ****************************** //
    /**
     * @brief Compose this rigid motion \f$f\f$ with given rigid motion \f$g\f$
     *  so \f$f \circ g = f(g(x))\f$ is obtained.
     * @param rm Rigid motion to compose with, it is also notated as \f$g\f$
     * @return Rigid motion resulting from composition. It can be notated
     *  as \f$f \circ g = f(g(x))\f$.
     */
    virtual RigidMotion compose(RigidMotion const rm) const;
    /**
     * @brief Like compose method but updating this rigid motion instead of
     *  returning a new one
     * @see rigidmotion::RigidMotion::compose(RigidMotion)
     */
    virtual void composeInPlace(RigidMotion const rm);
    /**
     * @brief Obtain the supertype of the rigid motion
     * @return Rigid motion supertype
     * @see rigidmotion::RigidMotion::SuperType
     */
    virtual SuperType findSuperType() const;
    /**
     * @brief Check if rigid motion has fixed points
     *
     * A rigid motion is said to have fixed points by applying Rouché-Frobenius
     *  theorem to check if the system \f$(I-A | C)\f$ can be solved or not.
     *  Thus, if \f$\mathrm{rank}(I-A) = \mathrm{rank}(I-A|C)\f$ the system
     *  has solution and hence fixed points. Otherwise, the system cannot be
     *  solved so there are no fixed points.
     *
     * @return True if rigid motion has fixed points, false otherwise
     */
    virtual bool hasFixedPoints() const;
    /**
     * @brief Obtain the type of the rigid motion
     * @return Rigid motion type
     * @see rigidmotion::RigidMotion::Type
     */
    virtual Type findType() const;
    /**
     * @brief Obtain the dimensionality of the associated invariant which can
     *  be either the set of fixed points or the associated invariant variety
     *  if there are no fixed points.
     *
     * The dimensionality \f$d\f$ of the associated invariant can be obtained
     *  computing as follows, where \f$n\f$ is the dimensionality of
     *  the space for which the rigid motion is defined:
     * \f[
     *  d = n - \mathrm{rank}\left(I-A\right)
     * \f]
     *
     * A dimension of \f$0\f$ means the associated invariant is a point, a
     *  dimension of \f$1\f$ means it is a line, a dimension of \f$2\f$ means
     *  it is a plane and for the general case a dimension of \f$d\f$ means
     *  the associated invariant is a \f$d\f$-dimensional hyperplane
     *
     * @return Dimensionality of the associated invariant
     */
    virtual size_t findInvariantDimensionality() const;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Get the translation column vector
     * @return Translation column vector of the rigid motion
     */
    inline colvec getC() const {return C;}
    /**
     * @brief Set the translation column vector
     * @param C New translation column vector for the rigid motion
     */
    inline void setC(colvec const C) {this->C = C;}
    /**
     * @brief Get the fixed origin transformation matrix
     * @return Fixed origin transformation matrix of the rigid motion
     */
    inline arma::mat getA() const {return A;}
    /**
     * @brief Set the fixed origin transformation matrix
     * @param A New fixed origin transformation matrix for the rigid motion
     */
    inline void setA(arma::mat const A) {this->A = A;}
    /**
     * @brief Get the dimensionality of the rigid motion
     * @return Rigid motion dimensionality
     */
    inline size_t getDimensionality() const {return this->A.n_rows;}

};

}