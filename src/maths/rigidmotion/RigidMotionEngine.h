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
    /**
     * @brief Compute the set of fixed points of given rigid motion.
     *
     * Computing the set of fixed points for a given rigid motion means
     *  solving  \f$(I-A)X = C\f$
     *
     * @param f Rigid motion which set of fixed points must be computed
     * @param[out] dim Where the dimensionality of the fixed points variety
     *  will be stored. If it is \f$0\f$, then the lineal variety is a point,
     *  if it is \f$1\f$ then the lineal variety is a line, if it is \f$2\f$
     *  then the lineal variety is a plane. Generalizing, dim is the
     *  dimensionality of the hyperplane of fixed points.
     * @param safe Specify if use safe mode or not. When safe mode is enabled,
     *  existence of fixed points will be checked. Please, disable safe mode
     *  only if you are sure the rigid motion has a set of fixed points.
     *  Otherwise, unexpected behaviors might occur. It is enabled by default.
     * @return When dimensionality is \f$0\f$, it returns the exact coordinates
     *  of the only fixed point. When dimensionality is \f$n\f$ (matches
     *  the full space dimensionality), then the matrix containing basis
     *  vectors will be returned. When dimensionality is \f$0<d<n\f$ the
     *  coefficients of implicit equation defining the lineal variety of fixed
     *  points will be returned.
     */
    mat computeFixedPoints(
        RigidMotion const &f,
        size_t &dim,
        bool safe=true
    );
};
}