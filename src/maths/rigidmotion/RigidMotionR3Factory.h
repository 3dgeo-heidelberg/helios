#pragma once

#include <rigidmotion/RigidMotionFactory.h>

#include <armadillo>

using namespace arma;

namespace rigidmotion{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for rigid motions in
 *  \f$\mathbb{R}^{3}\f$
 *
 * @see rigidmotion::RigidMotionFactory
 */
class RigidMotionR3Factory : public RigidMotionFactory{
protected:
    // ***  CONSTANTS  *** //
    // ******************* //
    /**
     * @brief Decimal precision tolerance
     */
    static double const eps;
    /**
     * @brief The Jordan normal form of a reflection in \f$\mathbb{R}^{3}\f$
     *
     * \f[
     *  J = \left[\begin{array}{lll}
     *      -1 & 0 & 0 \\
     *      0 & 1 & 0 \\
     *      0 & 0 & 1
     *  \end{array}\right]
     * \f]
     *
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     */
    static mat const canonicalReflection;

public:

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief RigidMotionR3Factory default constructor
     */
    RigidMotionR3Factory() = default;
    virtual ~RigidMotionR3Factory() = default;

    // ***  RIGID MOTION FACTORY METHODS  *** //
    // ************************************** //
    /**
     * @brief Implementation of identity rigid motion in \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionFactory::makeIdentity
     */
    RigidMotion makeIdentity() const override;
    /**
     * @brief Implementation of translation rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionFactory::makeTranslation
     */
    RigidMotion makeTranslation(colvec const shift) const override;
    /**
     * @brief Implementation of reflection rigid motion in \f$\mathbb{R}^{3}\f$
     *
     * Let \f$\vec{u}\f$ be the orthogonal vector defining the reflection
     *  plane. Now let \f$\vec{\alpha}\f$ be an orthogonal vector with respect
     *  to \f$\vec{u}\f$ so
     *  \f$\left\langle\vec{u}, \vec{\alpha}\right\rangle=0\f$. Then, another
     *  vector \f$\vec{\beta}\f$ orthogonal to both \f$\vec{u}\f$ and
     *  \f$\vec{\alpha}\f$ can be obtained through cross product as
     *  \f$\vec{\beta} = \vec{u} \times \vec{\alpha}\f$.
     *
     * Once vectors satisfying \f$
     *  \left\langle\vec{u}, \vec{\alpha}\right\rangle =
     *  \left\langle\vec{u}, \vec{\beta}\right\rangle =
     *  \left\langle\vec{\alpha}, \vec{\beta}\right\rangle = 0
     * \f$
     *  are known, the basis matrix \f$B\f$ can be defined as:
     *
     * \f[
     *  B = \left[\begin{array}{lll}
     *      u_x & \alpha_x & \beta_x \\
     *      u_y & \alpha_y & \beta_y \\
     *      u_z & \alpha_z & \beta_z
     *  \end{array}\right]
     * \f]
     *
     * Notice the Jordan normal form \f$J\f$ of a reflection in
     *  \f$\mathbb{R}^{3}\f$ corresponds to a plane having \f$e_1=(1, 0, 0)\f$
     *  as orthonormal vector. It is as shown below:
     *
     * \f[
     *  J = \left[\begin{array}{lll}
     *      -1 & 0 & 0 \\
     *      0 & 1 & 0 \\
     *      0 & 0 & 1
     *  \end{array}\right]
     * \f]
     *
     * In consequence, the affine application for a reflection over an
     *  arbitrary plane can be expressed as
     *  \f$Y = C + AX = \vec{0} + BJB^{-1}X\f$
     *
     * @param ortho Orthogonal vector defining the reflection plane
     * @return Reflection rigid motion in \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionR3Factory::canonicalReflection
     */
    virtual RigidMotion makeReflection(colvec const ortho) const ;
    /**
     * @brief Fast implementation of a reflection rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly it is recommended that input
     *  vector is normalized. Otherwise, output might lead to unexpected
     *  behaviors.
     *
     * This function is slower than
     *  makeReflectionFast(colvec const, colvec const, colvec const) but its
     *  input does not have as many constraints.
     *
     * @param orthonormal Orthonormal vector defining the reflection plane
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     * @see makeReflectionFast(colvec const, colvec const, colvec const)
     */
    virtual RigidMotion makeReflectionFast(colvec const orthonormal) const;
    /**
     * @brief Fast implementation of a reflection rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly it is recommended that all input
     *  vectors are given as a set of orthonormal vectors. Otherwise,
     *  output might lead to unexpected behaviors.
     *
     * This function is faster that makeReflectionFast(colvec const) but
     *  its input must satisfy more requirements.
     *
     * @param u Orthonormal vector defining the reflection plane
     * @param alpha First normal vector contained in the reflection plane
     * @param beta Second normal vector contained in the reflection plane
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     * @see makeReflectionFast(colvec const)
     */
    virtual RigidMotion makeReflectionFast(
        colvec const u,
        colvec const alpha,
        colvec const beta
    ) const;
    /**
     * @brief Implementation of a reflection rigid motion over a plane with
     *  \f$x\f$-axis as orthonormal.
     * @return Reflection rigid motion over \f$x=0\f$ plane in
     *  \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     */
    virtual RigidMotion makeReflectionX() const;
    /**
     * @brief Implementation of a reflection rigid motion over a plane with
     *  \f$y\f$-axis as orthonormal.
     * @return Reflection rigid motion over \f$y=0\f$ plane in
     *  \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     */
    virtual RigidMotion makeReflectionY() const;
    /**
     * @brief Implementation of a reflection rigid motion over a plane with
     *  \f$z\f$-axis as orthonormal.
     * @return Reflection rigid motion over \f$z=0\f$ plane in
     *  \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     */
    virtual RigidMotion makeReflectionZ() const;
    /**
     * @brief Implementation of a glide reflection or glide plane rigid motion
     *  in \f$\mathbb{R}^{3}\f$.
     *
     * Let \f$\vec{u}\f$ be the orthogonal vector defining the reflection plane
     *  so a reflection can be performed as documented in the
     *  makeReflection(colvec const) method. If \f$\vec{v}\f$ is a vector
     *  contained in the reflection plane, which means
     *  \f$\left\langle\vec{u},\vec{v}\right\rangle = 0\f$ must be satisfied.
     *  Then the affine application for the glide reflection can be expressed
     *  as \f$Y = \vec{v} + BJB^{-1}X\f$ where \f$B\f$ is the basis matrix
     *  as explained in makeReflection(colvec const).
     *
     * @param ortho Orthogonal vector defining the reflection plane
     * @param shift The translation vector. It must be orthogonal with respect
     *  to the plane orthogonal vector, otherwise exception will be thrown.
     *
     * @return Glide reflection (glide plane) rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     */
    virtual RigidMotion makeGlideReflection(
        colvec const ortho,
        colvec const shift
    ) const;
    /**
     * @brief Fast implementation of a reflection rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly it is recommended that input
     *  orthonormal vector is truly normalized. Otherwise, output might lead to
     *  unexpected behaviors.
     *
     * Also, restrictions with respect to shift vector must be satisfied as for
     *  the makeGlideReflection(colvec const, colvec const) method.
     *
     * @param orthonormal Orthonormal vector defining the reflection plane
     * @param shift Translation/glide vector. Must be orthogonal with respect
     *  to plane orthonormal vector
     * @see makeGlideReflection(colvec const, colvec const)
     */
    virtual RigidMotion makeGlideReflectionFast(
        colvec const orthonormal,
        colvec const shift
    ) const;
    /**
     * @brief Implementation of rotation rigid motion over an arbitrary axis
     *  in \f$\mathbb{R}^{3}\f$
     *
     * Let \f$\vec{u}\f$ be an arbitrary director vector defining the rotation
     *  axis. Now let
     *  \f$\hat{u} = \frac{\vec{u}}{\left\Vert\vec{u}\right\Vert}\f$ be its
     *  normalized form. Thus, the affine application for the rotation can be
     *  defined as \f$Y = \vec{0} + AX\f$ where \f$A\f$ is as follows:
     *
     * \f[
     *  A = \left[\begin{array}{lll}
     *      \cos(\theta) + \hat{u}_x^2(1-\cos(\theta)) &
     *      \hat{u}_x\hat{u}_y (1-\cos(\theta)) - \hat{u}_z\sin(\theta) &
     *      \hat{u}_x\hat{u}_z (1-\cos(\theta)) + \hat{u}_y\sin(\theta) \\
     *      \hat{u}_y\hat{u}_x(1-\cos(\theta)) + \hat{u}_z\sin(\theta) &
     *      \cos(\theta) + \hat{u}_y^2(1-\cos(\theta)) &
     *      \hat{u}_y\hat{u_z}(1-\cos(\theta)) - \hat{u}_x\sin(\theta) \\
     *      \hat{u}_z\hat{u}_x(1-\cos(\theta)) - \hat{u}_y\sin(\theta) &
     *      \hat{u}_z\hat{u}_y(1-\cos(\theta)) + \hat{u}_x\sin(\theta) &
     *      \cos(\theta) + \hat{u}_z^2(1-\cos(\theta))
     *  \end{array}\right]
     * \f]
     *
     * @param axis The rotation axis itself
     * @param theta The rotation angle
     * @return Rotation rigid motion in \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeRotation(
        colvec const axis,
        double const theta
    ) const;
    /**
     * @brief Fast implementation of a rotation rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly, it is recommended that input
     *  rotation axis is normalized. Otherwise, output might lead to unexpected
     *  behaviors.
     *
     * @param axis Normalized rotation axis
     * @param theta Rotation angle
     * @see RigidMotionR3Factory::makeRotation(colvec const, double const)
     */
    virtual RigidMotion makeRotationFast(
        colvec const axis,
        double const theta
    ) const;
    /**
     * @brief Implementation of a rotation rigid motion over the \f$x\f$-axis.
     * @param theta The rotation angle
     * @return Rotation rigid motion over \f$e_1\f$ vector (\f$x\f$-axis) in
     *  \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeRotationX(double const theta) const;
    /**
     * @brief Implementation of a rotation rigid motion over the \f$y\f$-axis.
     * @param theta The rotation angle
     * @return Rotation rigid motion over \f$e_2\f$ vector (\f$y\f$-axis) in
     *  \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeRotationY(double const theta) const;
    /**
     * @brief Implementation of a rotation rigid motion over the \f$z\f$-axis.
     * @param theta The rotation angle
     * @return Rotation rigid motion over \f$e_3\f$ vector (\f$z\f$-axis) in
     *  \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeRotationZ(double const theta) const;
    /**
     * @brief Implementation of an helical rigid motion over an arbitrary axis
     *  in \f$\mathbb{R}^3\f$.
     *
     * First a rotation is performed as in the
     *  makeRotation(colvec const, double const) method. Then a transposition
     *  parallel to the rotation axis is applied. Thus, the affine application
     *  can be defines as \f$Y = C + AX\f$ where \f$C\f$ is the transposition
     *  column-vector and \f$A\f$ is the rotation matrix described in
     *  makeRotation(colvec const, double const).
     *
     * @param axis Rotation axis
     * @param theta Rotation angle
     * @param glide How many glide apply in the direction of rotation axis
     *  afther the rotation
     * @return Helical rigid motion in \f$\mathbb{R}^{3}\f$
     * @see RigidMotionR3Factory::makeRotation(colvec const, double const)
     */
    virtual RigidMotion makeHelical(
        colvec const axis,
        double const theta,
        double const glide
    ) const;
    /**
     * @brief Fast implementation of a helical rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly, it is recommended that input
     *  rotation axis is normalized. Otherwise, output might lead to unexpected
     *  behaviors.
     *
     * @param axis Normalized rotation axis
     * @param theta Rotation angle
     * @param glide How many glide apply in the direction of rotation axis
     *  after the rotation
     * @see makeHelical(colvec const, double const, double const)
     */
    virtual RigidMotion makeHelicalFast(
        colvec const axis,
        double const theta,
        double const glide
    ) const;
    /**
     * @brief Implementation of a helical rigid motion over the \f$x\f$-axis.
     * @param theta The rotation angle
     * @param glide Glide amount to be applied in rotation axis direction
     * @return Helical rigid motion over \f$e_1\f$ vector (\f$x\f$-axis) in
     *  \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeHelicalX(
        double const theta,
        double const glide
    ) const;
    /**
     * @brief Implementation of a helical rigid motion over the \f$y\f$-axis.
     * @param theta The rotation angle
     * @param glide Glide amount to be applied in rotation axis direction
     * @return Helical rigid motion over \f$e_2\f$ vector (\f$y\f$-axis) in
     *  \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeHelicalY(
        double const theta,
        double const glide
    ) const;
    /**
     * @brief Implementation of a helical rigid motion over the \f$z\f$-axis.
     * @param theta The rotation angle
     * @param glide Glide amount to be applied in rotation axis direction
     * @return Helical rigid motion over \f$e_3\f$ vector (\f$z\f$-axis) in
     *  \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeHelicalZ(
        double const theta,
        double const glide
    ) const;
    /**
     * @brief Implementation of rotational symmetry rigid motion over an
     *  arbitrary rotation axis in \f$\mathbb{R}^{3}\f$
     *
     * Let \f$\vec{u}\f$ be an arbitrary director vector defining the rotation
     *  axis so the symmetry/reflection plane is \f$\pi = \left\{(x, y, z) :
     *  \lambda\vec{\alpha} + \mu\vec{\beta}\right\}\f$ where
     *  \f$\left\langle{\vec{u}, \vec{\alpha}}\right\rangle =
     *  \left\langle{\vec{u}, \vec{\beta}}\right\rangle =
     *  \left\langle{\vec{\alpha}, \vec{\beta}}\right\rangle =
     *  0\f$. It is \f$\pi \perp \vec{u}\f$. Thus, the rotational symmetry
     *  can be understood as a rotation around \f$\vec{u}\f$ followed by
     *  a reflection over \f$\pi\f$ as the symmetry plane which is orthogonal
     *  to the rotation axis.
     *
     * In consequence, the affine application can be defined as
     *  \f$Y = \vec{0} + AX\f$ where \f$A = BJB^{-1}R\f$. In this case,
     *  \f$R\f$ would be the arbitrary rotation matrix in \f$\mathbb{R}^{3}\f$
     *  as defined for the makeRotation(colvec const, double const) method
     *  while \f$BJB^{-1}\f$ is the arbitrary reflection matrix in
     *  \f$\mathbb{R}^{3}\f$ as defined for the makeReflection(colvec const)
     *  method.
     *
     * @param axis Rotation axis which is also the orthogonal vector for the
     *  symmetry/reflection plane
     * @param theta Rotation angle
     * @return Rotational symmetry rigid motion in \f$\mathbb{R}^{3}\f$
     * @see RigidMotionR3Factory::makeRotation(colvec const, double const)
     * @see RigidMotionR3Factory::makeReflection(colvec const)
     * @see makeRotationalSymmetry(colvec const, double const, colvec const)
     */
    virtual RigidMotion makeRotationalSymmetry(
        colvec const axis,
        double const theta
    ) const;
    /**
     * @brief Fast implementation of a rotational symmetry rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly, it is recommended that input
     *  rotation axis is normalized. Otherwise, output might lead to unexpected
     *  behaviors.
     *
     * @param axis Normalized rotation axis which is also the orthonormal
     *  vector for the symmetry/reflection plane
     * @param theta Rotation angle
     * @see makeRotationalSymmetry(colvec const, double const)
     */
    virtual RigidMotion makeRotationalSymmetryFast(
        colvec const axis,
        double const theta
    ) const;
    /**
     * @brief Implementation of rotational symmetry rigid motion over an
     *  arbitrary rotation axis in \f$\mathbb{R}^{3}\f$
     *
     * The rotation center implies redefining \f$C\f$ for the affine
     *  application as follows, where \f$Q\f$ is the rotation center:
     *
     * \f[
     *  Y = C + AX = (I-A)Q + AX
     * \f]
     *
     * @param axis Rotation axis which is also the orthogonal vector for the
     *  symmetry/reflection plane
     * @param theta Rotation angle
     * @param center The center of rotational symmetry where the rotation
     *  axis and the reflection plane intersect
     * @see makeRotationalSymmetry(colvec const, double const)
     */
    virtual RigidMotion makeRotationalSymmetry(
        colvec const axis,
        double const theta,
        colvec const center
    ) const;
    /**
     * @brief Fast implementation of a rotational symmetry rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly, it is recommended that input
     *  rotation axis is normalized. Otherwise, output might lead to unexpected
     *  behaviors.
     *
     * @param axis Rotation axis which is also the orthogonal vector for the
     *  symmetry/reflection plane
     * @param theta Rotation angle
     * @param center The center of rotational symmetry where the rotation
     *  axis and the reflection plane intersect
     * @see makeRotationalSymmetry(colvec const, double const, colvec const)
     */
    virtual RigidMotion makeRotationalSymmetryFast(
        colvec const axis,
        double const theta,
        colvec const center
    ) const;
    /**
     * @brief Implementation of a rotational symmetry over the \f$x\f$-axis
     * @param theta Rotation angle
     * @return Rotational symmetry rigid motion over \f$e_1\f$ vector
     *  (\f$x\f$-axis) in \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeRotationalSymmetryX(double const theta) const;
    /**
     * @brief Implementation of a rotational symmetry over the \f$y\f$-axis
     * @param theta Rotation angle
     * @return Rotational symmetry rigid motion over \f$e_2\f$ vector
     *  (\f$y\f$-axis) in \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeRotationalSymmetryY(double const theta) const;
    /**
     * @brief Implementation of a rotational symmetry over the \f$z\f$-axis
     * @param theta Rotation angle
     * @return Rotational symmetry rigid motion over \f$e_3\f$ vector
     *  (\f$z\f$-axis) in \f$\mathbb{R}^{3}\f$
     */
    virtual RigidMotion makeRotationalSymmetryZ(double const theta) const;
};
}