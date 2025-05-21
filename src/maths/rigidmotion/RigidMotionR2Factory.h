#pragma once

#include <rigidmotion/RigidMotionFactory.h>

#include <armadillo>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/void_cast.hpp>

namespace rigidmotion {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing building methods for rigid motions in
 *  \f$\mathbb{R}^{2}\f$.
 *
 * @see rigidmotion::RigidMotionFactory
 */
class RigidMotionR2Factory : public RigidMotionFactory
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a \f$\mathbb{R}^{2}\f$ rigid motion factory to a
   *  stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the \f$\mathbb{R}^{2}\f$ rigid motion
   *  factory
   */
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::void_cast_register<RigidMotionR2Factory,
                                             RigidMotionFactory>();
    ar& boost::serialization::base_object<RigidMotionFactory>(*this);
  }

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief RigidMotionR2Factory default constructor
   */
  RigidMotionR2Factory() = default;
  ~RigidMotionR2Factory() override = default;

  // ***  RIGID MOTION FACTORY METHODS  *** //
  // ************************************** //
  /**
   * @brief Implementation of identity rigid motion in \f$\mathbb{R}^{2}\f$
   * @see rigidmotion::RigidMotionFactory::makeIdentity
   */
  RigidMotion makeIdentity() const override;
  /**
   * @brief Implementation of translation rigid motion in
   *  \f$\mathbb{R}^{2}\f$
   * @see rigidmotion::RigidMotionFactory::makeTranslation
   */
  RigidMotion makeTranslation(arma::colvec const shift) const override;
  /**
   * @brief Implementation of reflection rigid motion in \f$\mathbb{R}^{2}\f$
   *
   * Let \f$\vec{u}\f$ be an arbitrary axis and let \f$\theta\f$ be the angle
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
   * @return Reflection rigid motion in \f$\mathbb{R}^{2}\f$
   */
  virtual RigidMotion makeReflection(arma::colvec const axis) const;
  /**
   * @brief As makeReflection method but receiving \f$\theta\f$ as the angle
   *  between reflection axis and \f$e_1\f$ instead of the reflection axis
   *  itself
   * @param theta Angle between \f$e_1\f$ (unitary vector for x-axis in
   *  canonical basis) and rotation axis
   * @see rigidmotion::RigidMotionR2Factory::makeReflection(colvec)
   */
  virtual RigidMotion makeReflection(double const theta) const;
  /**
   * @brief Implementation of glide reflection rigid motion in
   *  \f$\mathbb{R}^{2}\f$
   *
   * Let \f$\vec{u}\f$ be an arbitrary axis, let \f$\theta\f$ be the angle
   *  between \f$\vec{u}\f$ and \f$e_1\f$ and let \f$\lambda\f$ specify
   *  how many glide in the \f$\hat{u}\f$ axis direction will be applied after
   *  the reflection. Notice
   *  \f$\hat{u} = \frac{\vec{u}}{\Vert\vec{u}\Vert}\f$, it is
   *  \f$\vec{u}\f$ as normalized director vector.
   *  Thus, the affine application for the glide reflection
   *  would be \f$Y = \lambda\hat{u} + AX\f$ where \f$A\f$ is as documented
   *  for the makeReflection method.
   *
   * @param axis Reflection axis
   * @param glide How many glide apply in the direction of reflection axis
   *  afther the reflection
   * @return Glide reflection rigid motion
   * @see rigidmotion::RigidMotionR2Factory::makeReflection(arma::colvec)
   */
  virtual RigidMotion makeGlideReflection(arma::colvec const axis,
                                          double const glide) const;
  /**
   * @brief As makeGlideReflection method but receiving \f$\theta\f$ as the
   *  angle between reflection axis and \f$e_1\f$ instead of the reflection
   *  axis itself
   *
   * @param theta Angle between \f$e_1\f$ (unitary vector for x-axis in
   *  canonical basis) and rotation axis
   * @param glide How many glide apply in the direction of reflection axis
   *  afther the reflection
   * @see RigidMotionR2Factory::makeGlideReflection(colvec, double)
   */
  virtual RigidMotion makeGlideReflection(double const theta,
                                          double const glide) const;
  /**
   * @brief Implementation of rotation rigid motion in \f$\mathbb{R}^{2}\f$
   *
   * Let \f$\theta\f$ be the rotation angle and \f$Q=(Q_x, Q_y)\f$ be the
   *  rotation center. Now, the affine application for the rotation would be
   *  \f$Y = C + AX\f$ where:
   *
   * \f[
   *  A = \left[\begin{array}{ll}
   *      \cos(\theta) & -\sin(\theta) \\
   *      \sin(\theta) & \cos(\theta)
   *  \end{array}\right]
   * \f]
   *
   * and
   *
   * \f[
   *  C = (I-A)Q = \left[\begin{array}{ll}
   *      1-\cos(\theta) & \sin(\theta) \\
   *      -\sin(\theta) & 1-\cos(\theta)
   *  \end{array}\right]
   *  \left[\begin{array}{l}
   *      Q_x \\
   *      Q_y
   *  \end{array}\right]
   * \f]
   *
   * @param theta The rotation angle
   * @param center The rotation center point
   * @return Rotation rigid motion in \f$\mathbb{R}^{2}\f$
   */
  virtual RigidMotion makeRotation(double const theta,
                                   arma::colvec const center) const;
};

}
