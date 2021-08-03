#pragma once

#include <maths/rigidmotion/RigidMotion.h>

#include <armadillo>

using namespace arma;

namespace rigidmotion{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class to handle operations with rigid motions
 *
 * @see rigidmotion::RigidMotion
 */
class RigidMotionEngine {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief RigidMotionEngine default constructor
     */
    RigidMotionEngine() = default;
    virtual ~RigidMotionEngine() = default;

    // ***  RIGID MOTION ENGINE METHODS  *** //
    // ************************************* //
    /**
     * @brief Apply rigid motion \f$f\f$ to a matrix of points \f$X\f$
     *
     * Let \f$f(X) = C + AX\f$ be a rigid motion defined for points in
     *  \f$\mathbb{R}^{n}\f$. Now let \f$X_{n \times m}\f$ be a matrix with
     *  as many rows as the space dimensionality and as many columns as points
     *  to be transformed. Thus, the output of this function is a matrix
     *  \f$Y_{n \times m} = f(X)\f$ containing all transformed points.
     *
     * Notice the transposition as column vector \f$C\f$ will be applied to
     *  each column of \f$AX\f$.
     *
     * @param f The rigid motion to be applied
     * @param X The matrix of points to be transformed
     * @return Result of applying the rigid motion to the matrix of points
     */
    mat apply(RigidMotion const &f, mat const X);
    /**
     * @brief Apply rigid motion \f$f\f$ to a point \f$X\f$
     *
     * Let \f$f(X) = C + AX\f$ be a rigid motion defined for points in
     *  \f$\mathbb{R}^{n}\f$. Now let \f$X\f$ be a column vector representing
     *  a point in a \f$n\f$-dimensional space. Therefore, the output of this
     *  function is a column vector \f$Y = f(X)\f$ corresponding to the
     *  transformed point.
     *
     * @param f The rigid motion to be applied
     * @param X The point as column vector to be transformed
     * @return Result of applying the rigid motion to the point
     */
    colvec apply(RigidMotion const &f, colvec const X);
    /**
     * @brief Compose given rigid motions: \f$f \circ g = f(g(X))\f$
     *
     * @param f The rigid motion in second place of composition
     * @param g The rigid motion in first place of composition
     *
     * @return Composition of given rigid motions \f$f \circ g = f(g(X))\f$
     */
    RigidMotion compose(RigidMotion const &f, RigidMotion const &g);
};
}