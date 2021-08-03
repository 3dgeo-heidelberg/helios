#pragma once

#include <armadillo>

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
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The column vector representing the translation
     */
    colvec C;
    /**
     * @brief The matrix representing the fixed origin transformation
     */
    mat A;

public:
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
    RigidMotion(colvec const C, mat const A) : C(C), A(A) {};
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
    inline mat getA() const {return A;}
    /**
     * @brief Set the fixed origin transformation matrix
     * @param A New fixed origin transformation matrix for the rigid motion
     */
    inline void setA(mat const A) {this->A = A;}

};

}