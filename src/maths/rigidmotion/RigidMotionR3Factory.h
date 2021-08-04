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
    RigidMotion makeIdentity() override;
    /**
     * @brief Implementation of translation rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     * @see rigidmotion::RigidMotionFactory::makeTranslation
     */
    RigidMotion makeTranslation(colvec const shift) override;
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
    virtual RigidMotion makeReflection(colvec const ortho);
    /**
     * @brief Fast implementation of a reflection rigid motion in
     *  \f$\mathbb{R}^{3}\f$
     *
     * For this function to work properly it is recommended that input
     *  vector is normalized. Otherwise, output might lead to unexpected
     *  behaviors.
     *
     * This function is slower than makeReflectionFast(colvec const, colvec
     *  const, colvec const) but its input does not have as many constraints.
     *
     * @param orthonormal Orthonormal vector defining the reflection plane
     * @see rigidmotion::RigidMotionR3Factory::makeReflection(colvec const)
     * @see makeReflectionFast(colvec const, colvec const, colvec const)
     */
    virtual RigidMotion makeReflectionFast(colvec const orthonormal);
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
    );

};
}