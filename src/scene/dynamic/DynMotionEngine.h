#pragma once

#include <rigidmotion/RigidMotionEngine.h>
#include <rigidmotion/RigidMotionR3Factory.h>
#include <DynObject.h>
#include <DynMotion.h>

using namespace arma;
using rigidmotion::RigidMotionEngine;
using rigidmotion::RigidMotionR3Factory;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Adapter which wraps a rigid motion engine to make it fit the dynamic
 *  Helios context.
 * @see rigidmotion::RigidMotionEngine
 * @see DynObject
 * @see DynMotion
 * @see DynMovingObject
 * @see DynSequentiableMovingObject
 */
class DynMotionEngine {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The rigid motion engine which is the core of the dynamic motion
     *  engine
     * @see rigidmotion::RigidMotionEngine
     */
    RigidMotionEngine rme;
    /**
     * @brief The factory for rigid motions in \f$\mathbb{R}^3\f$ of the
     *  dynamic motion engine
     * @see rigidmotion::RigidMotionR3Factory
     */
    RigidMotionR3Factory rm3f;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for dynamic motion engine. It uses a default
     *  rigid motion engine
     */
    DynMotionEngine() = default;
    /**
     * @brief Construct a dynamic motion engine from given rigid motion engine
     * @param rme Rigid motion engine composing the dynamic motion engine
     */
    DynMotionEngine(RigidMotionEngine const &rme) : rme(rme) {}
    virtual ~DynMotionEngine() = default;

    // ***  DYNAMIC MOTION ENGINE METHODS  *** //
    // *************************************** //
    /**
     * @brief Apply dynamic motion \f$f\f$ to a matrix of points \f$X\f$
     *  belonging to given dynamic object.
     *
     * To explain different applications let \f$o\f$ be the centroid of the
     *  dynamic object.
     *
     * If the dynamic motion has no modes enabled, it will be applied
     *  straight forward to both \f$X\f$ and \f$O\f$.
     *
     * If the dynamic motion has self mode enabled but not normal mode enabled,
     *  it will be applied to both \f$X\f$ and \f$O\f$. However, when applied
     *  to \f$X\f$ instead of computing \f$f(X) = C + AX\f$ it will compute
     *  \f$f(X) = O + C + A(X-O)\f$. On the other hand, updating \f$O\f$ leads
     *  to \f$f(O) = O + C + A(O-O) = O + C\f$
     *
     * If the dynamic motion has both self mode and normal mode enabled, then
     *  it will be applied to \f$X\f$ but not to \f$O\f$, using the basic
     *  affine application \f$f(X) = C + AX\f$.
     *
     * If the dynamic motion hast self mode disabled but normal mode enabled,
     *  then it will behave exactly as if both self mode and normal mode were
     *  enabled.
     *
     * @param f Dynamic motion to be applied
     * @param X Matrix of points to be transformed
     * @param dynObj Dynamic object associated with the matrix of points. It
     *  will be updated according to given dynamic motion, if necessary
     * @return Result of applying the dynamic motion to the matrix of points
     * @see
     *  rigidmotion::RigidMotionEngine::apply(RigidMotion const &, mat const &)
     * @see DynMotion::selfMode
     * @see DynMotion::normalMode
     */
    mat apply(DynMotion const &f, mat const &X, DynObject &dynObj);
    /**
     * @brief Compose given dynamic motions: \f$f \circ g = f(g(X))\f$
     *  considering they belong to given dynamic object
     * @param f The dynamic motion in second place of composition
     * @param g The dynamic motion in first place of composition
     * @param dynObj Dynamic object associated with given dynamic motions
     *  to be composed
     * @return Composition of given dynamic motions \f$f \circ g = f(g(X))\f$
     */
    DynMotion compose(
        DynMotion const &f,
        DynMotion const &g,
        DynObject const &dynObj
    );
};