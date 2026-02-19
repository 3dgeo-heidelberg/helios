#pragma once

#include <maths/rigidmotion/RigidMotion.h>

namespace rigidmotion {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class that must be extended by any class which provides
 *  factory methods for rigid motions. Notice all rigid motions should be
 *  instantiated through corresponding factories.
 *
 * @see rigidmotion::RigidMotionR2Factory
 * @see rigidmotion::RigidMotionR3Factory
 */
class RigidMotionFactory
{
private:
  // *********************** //

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief RigidMotionFactory default constructor
   */
  RigidMotionFactory() = default;
  virtual ~RigidMotionFactory() = default;

  // ***  RIGID MOTION FACTORY METHODS  *** //
  // ************************************** //
  /**
   * @brief Build the identity rigid motion.
   *
   * The identity rigid motion \f$f\f$ satisfies \f$f(x)=x\f$. Using affine
   *  application notation it is \f$X = C + AX\f$ which is satisfied if
   *  \f$C=\vec{0}\f$ and \f$A=I\f$ where \f$I\f$ is the identity matrix.
   *
   * @return Identity rigid motion
   */
  virtual RigidMotion makeIdentity() const = 0;
  /**
   * @brief Build the translation rigid motion.
   *
   * The translation rigid motion \f$f\f$ satisfies \f$f(x) = x + \vec{s}\f$
   *  where \f$\vec{s}\f$ is the translation vector (shift). Using affine
   *  application notation it is \f$Y = \vec{s} + IX\f$ where \f$I\f$ is
   *  the identity matrix.
   *
   * @param shift Shift vector defining the translation
   * @return Translation rigid motion
   */
  virtual RigidMotion makeTranslation(arma::colvec const shift) const = 0;
};

}
