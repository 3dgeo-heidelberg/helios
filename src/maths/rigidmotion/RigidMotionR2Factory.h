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
 *  \f$\mathbb{R}^{2}\f$.
 *
 * @see rigidmotion::RigidMotionFactory
 */
class RigidMotionR2Factory : public RigidMotionFactory{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief RigidMotionR2Factory default constructor
     */
    RigidMotionR2Factory() = default;
    virtual ~RigidMotionR2Factory() = default;

    // ***  RIGID MOTION FACTORY METHODS  *** //
    // ************************************** //
    /**
     * @brief Implementation of identity rigid motion in \f$\mathbb{R}^{2}\f$
     * @see rigidmotion::RigidMotionFactory::makeIdentity
     */
    RigidMotion makeIdentity() override;
    /**
     * @brief Implementation of translation rigid motion in
     *  \f$\mathbb{R}^{2}\f$
     * @see rigidmotion::RigidMotionFactory::makeTranslation
     */
    RigidMotion makeTranslation(colvec const shift) override;
    /**
     * @brief Implementation of reflection rigid motion in \f$\mathbb{R}^{2}\f$
     *
     * Let \f$\vec{u}\f$ be an arbitrary axis and let \f$theta\f$ be the angle
     *  between \f$\vec{u}\f$ and \f$e_1\f$. Thus, the affine application for
     *  the reflection would be \f$Y = \vec{0} + AX\f$ where \f$A\f$ is:
     *
     * \f[
     *  A = \left[\begin{array}{ll}
     *      \cos(2\theta) & \sin(2\theta) \\
     *      \sin(2\theta) & -\cos(2\theta)
     *  \end{array}\right]
     * \f]
     *
     * @param axis Reflection axis
     * @return Reflexion rigid motion in \f$\mathbb{R}^{2}\f$
     */
    virtual RigidMotion makeReflection(colvec const axis);
    /**
     * @brief As makeReflection method but receiving \f$\theta\f$ as the angle
     *  between reflection axis and \f$e_1\f$ instead of the reflection axis
     *  itself
     * @param theta Angle between \f$e_1\f$ (unitary vector for x-axis in
     *  canonical basis) and rotation axis
     * @see rigidmotion::RigidMotionR2Factory::makeReflection(colvec)
     */
    virtual RigidMotion makeReflection(double const theta);
};

}