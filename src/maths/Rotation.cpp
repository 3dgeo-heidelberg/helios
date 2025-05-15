#include <iostream>

#include <glm/gtx/norm.hpp>
#include <glm/gtx/perpendicular.hpp>
#include <logging.hpp>

#include "HeliosException.h"
#include "MathConstants.h"
#include "Rotation.h"

/** Build a rotation from the quaternion coordinates.
 * <p>A rotation can be built from a <em>normalized</em> quaternion,
 * i.e. a quaternion for which q<sub>0</sub><sup>2</sup> +
 * q<sub>1</sub><sup>2</sup> + q<sub>2</sub><sup>2</sup> +
 * q<sub>3</sub><sup>2</sup> = 1. If the quaternion is not normalized,
 * the constructor can normalize it in a preprocessing step.</p>
 * <p>Note that some conventions put the scalar part of the quaternion
 * as the 4<sup>th</sup> component and the vector part as the first three
 * components. This is <em>not</em> our convention. We put the scalar part
 * as the first component.</p>
 * @param q0 scalar part of the quaternion
 * @param q1 first coordinate of the vectorial part of the quaternion
 * @param q2 second coordinate of the vectorial part of the quaternion
 * @param q3 third coordinate of the vectorial part of the quaternion
 * @param needsNormalization if true, the coordinates are considered
 * not to be normalized, a normalization preprocessing step is performed
 * before using them
 */
Rotation::Rotation(double q0,
                   double q1,
                   double q2,
                   double q3,
                   bool needsNormalization)
{

  if (needsNormalization) {
    // normalization preprocessing
    double inv = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= inv;
    q1 *= inv;
    q2 *= inv;
    q3 *= inv;
  }

  this->q0 = q0;
  this->q1 = q1;
  this->q2 = q2;
  this->q3 = q3;
}

/** Build a rotation from an axis and an angle.
 * <p>We use the convention that angles are oriented according to
 * the effect of the rotation on vectors around the axis. That means
 * that if (i, j, k) is a direct frame and if we first provide +k as
 * the axis and &pi;/2 as the angle to this constructor, and then
 * {@link #applyTo(Vector3D) apply} the instance to +i, we will get
 * +j.</p>
 * <p>Another way to represent our convention is to say that a rotation
 * of angle &theta; about the unit vector (x, y, z) is the same as the
 * rotation build from quaternion components { cos(-&theta;/2),
 * x * sin(-&theta;/2), y * sin(-&theta;/2), z * sin(-&theta;/2) }.
 * Note the minus sign on the angle!</p>
 * <p>On the one hand this convention is consistent with a vectorial
 * perspective (moving vectors in fixed frames), on the other hand it
 * is different from conventions with a frame perspective (fixed vectors
 * viewed from different frames) like the ones used for example in spacecraft
 * attitude community or in the graphics community.</p>
 * @param axis axis around which to rotate
 * @param angle rotation angle.
 * @exception MathIllegalArgumentException if the axis norm is zero
 */
Rotation::Rotation(glm::dvec3 axis, double angle)
{

  double norm = glm::l2Norm(axis);
  if (norm == 0) {
    logging::WARN("ERROR: ZERO_NORM_FOR_ROTATION_AXIS");
  }

  double halfAngle = -0.5 * angle;
  double coeff = sin(halfAngle) / norm;

  q0 = cos(halfAngle);
  q1 = coeff * axis.x;
  q2 = coeff * axis.y;
  q3 = coeff * axis.z;
}

/** Build one of the rotations that transform one vector into another one.

* <p>Except for a possible scale factor, if the instance were
* applied to the vector u it will produce the vector v. There is an
* infinite number of such rotations, this constructor choose the
* one with the smallest associated angle (i.e. the one whose axis
* is orthogonal to the (u, v) plane). If u and v are colinear, an
* arbitrary rotation axis is chosen.</p>

* @param u originWaypoint vector
* @param v desired image of u by the rotation
* @exception MathArithmeticException if the norm of one of the vectors is zero
*/
Rotation::Rotation(glm::dvec3 u, glm::dvec3 v)
{

  double normProduct = glm::l2Norm(u) * glm::l2Norm(v);
  if (normProduct == 0) {
    throw HeliosException(
      "Rotation::Rotation(glm::dvec3, glm::dvec3 ZERO NORM FOR ROTATION "
      "DEFINING VECTOR EXCEPTION");
  }

  double dot = glm::dot(u, v);

  if (dot < ((2.0e-15 - 1.0) * normProduct)) {
    // special case u = -v: we select a PI angle rotation around
    // an arbitrary vector orthogonal to u
    // Check also glm::perp function to compute w as orthogonal to u
    glm::vec3 nu = glm::normalize(u);
    glm::vec3 nv = glm::vec3(nu.z, nu.x, nu.y);
    glm::dvec3 w = glm::normalize(glm::cross(nu, nv));

    q0 = 0.0;
    q1 = -w.x;
    q2 = -w.y;
    q3 = -w.z;
  } else {
    // general case: (u, v) defines a plane, we select
    // the shortest possible rotation: axis orthogonal to this plane
    q0 = sqrt(0.5 * (1.0 + dot / normProduct));
    double coeff = 1.0 / (2.0 * q0 * normProduct);
    glm::dvec3 q = cross(v, u);
    q1 = coeff * q.x;
    q2 = coeff * q.y;
    q3 = coeff * q.z;
  }
}

/** Build a rotation from three Cardan or Euler elementary rotations.

* <p>Cardan rotations are three successive rotations around the
* canonical axes X, Y and Z, each axis being used once. There are
* 6 such sets of rotations (XYZ, XZY, YXZ, YZX, ZXY and ZYX). Euler
* rotations are three successive rotations around the canonical
* axes X, Y and Z, the first and last rotations being around the
* same axis. There are 6 such sets of rotations (XYX, XZX, YXY,
* YZY, ZXZ and ZYZ), the most popular one being ZXZ.</p>
* <p>Beware that many people routinely use the term Euler angles even
* for what really are Cardan angles (this confusion is especially
* widespread in the aerospace business where Roll, Pitch and Yaw angles
* are often wrongly tagged as Euler angles).</p>

* @param order order of rotations to use
* @param alpha1 angle of the first elementary rotation
* @param alpha2 angle of the second elementary rotation
* @param alpha3 angle of the third elementary rotation
*/
Rotation::Rotation(RotationOrder order,
                   double alpha1,
                   double alpha2,
                   double alpha3)
{
  Rotation r1 = Rotation(order.getA1(), alpha1);
  Rotation r2 = Rotation(order.getA2(), alpha2);
  Rotation r3 = Rotation(order.getA3(), alpha3);
  Rotation composed = r1.applyTo(r2.applyTo(r3));
  q0 = composed.q0;
  q1 = composed.q1;
  q2 = composed.q2;
  q3 = composed.q3;
}

/** Revert a rotation.
 * Build a rotation which reverse the effect of another
 * rotation. This means that if r(u) = v, then r.revert(v) = u. The
 * instance is not changed.
 * @return a new rotation whose effect is the reverse of the effect
 * of the instance
 */
Rotation
Rotation::revert()
{
  return Rotation(-q0, q1, q2, q3, false);
}

/** Get the normalized axis of the rotation.
 * @return normalized axis of the rotation
 * @see #Rotation(Vector3D, double)
 */
glm::dvec3
Rotation::getAxis()
{
  double squaredSine = q1 * q1 + q2 * q2 + q3 * q3;
  if (squaredSine == 0) {
    return glm::dvec3(1, 0, 0);
  } else if (q0 < 0) {
    double inverse = 1 / sqrt(squaredSine);
    return glm::dvec3(q1 * inverse, q2 * inverse, q3 * inverse);
  }
  double inverse = -1 / sqrt(squaredSine);
  return glm::dvec3(q1 * inverse, q2 * inverse, q3 * inverse);
}

/** Get the angle of the rotation.
 * @return angle of the rotation (between 0 and &pi;)
 * @see #Rotation(Vector3D, double)
 */
double
Rotation::getAngle()
{
  if ((q0 < -0.1) || (q0 > 0.1)) {
    return 2 * asin(sqrt(q1 * q1 + q2 * q2 + q3 * q3));
  } else if (q0 < 0) {
    return 2 * acos(-q0);
  }
  return 2 * acos(q0);
}

/** Get the 3X3 matrix corresponding to the instance
 * @return the matrix corresponding to the instance
 */
double**
Rotation::getMatrix()
{

  // products
  double q0q0 = q0 * q0;
  double q0q1 = q0 * q1;
  double q0q2 = q0 * q2;
  double q0q3 = q0 * q3;
  double q1q1 = q1 * q1;
  double q1q2 = q1 * q2;
  double q1q3 = q1 * q3;
  double q2q2 = q2 * q2;
  double q2q3 = q2 * q3;
  double q3q3 = q3 * q3;

  // create the matrix
  double** m = new double*[3];
  m[0] = new double[3];
  m[1] = new double[3];
  m[2] = new double[3];

  m[0][0] = 2.0 * (q0q0 + q1q1) - 1.0;
  m[1][0] = 2.0 * (q1q2 - q0q3);
  m[2][0] = 2.0 * (q1q3 + q0q2);

  m[0][1] = 2.0 * (q1q2 + q0q3);
  m[1][1] = 2.0 * (q0q0 + q2q2) - 1.0;
  m[2][1] = 2.0 * (q2q3 - q0q1);

  m[0][2] = 2.0 * (q1q3 - q0q2);
  m[1][2] = 2.0 * (q2q3 + q0q1);
  m[2][2] = 2.0 * (q0q0 + q3q3) - 1.0;

  return m;
}

/** Apply the rotation to a vector.
 * @param u vector to apply the rotation to
 * @return a new vector which is the image of u by the rotation
 */
glm::dvec3
Rotation::applyTo(glm::dvec3 u)
{
  double x = u.x;
  double y = u.y;
  double z = u.z;

  double s = q1 * x + q2 * y + q3 * z;

  return glm::dvec3(2 * (q0 * (x * q0 - (q2 * z - q3 * y)) + s * q1) - x,
                    2 * (q0 * (y * q0 - (q3 * x - q1 * z)) + s * q2) - y,
                    2 * (q0 * (z * q0 - (q1 * y - q2 * x)) + s * q3) - z);
}

/** Apply the rotation to a vector stored in an array.
 * @param in an array with three items which stores vector to rotate
 * @param out an array with three items to put result to (it can be the same
 * array as in)
 */
void
Rotation::applyTo(double* in, double* out)
{

  const double x = in[0];
  const double y = in[1];
  const double z = in[2];

  const double s = q1 * x + q2 * y + q3 * z;

  out[0] = 2 * (q0 * (x * q0 - (q2 * z - q3 * y)) + s * q1) - x;
  out[1] = 2 * (q0 * (y * q0 - (q3 * x - q1 * z)) + s * q2) - y;
  out[2] = 2 * (q0 * (z * q0 - (q1 * y - q2 * x)) + s * q3) - z;
}

/** Apply the inverse of the rotation to a vector.
 * @param u vector to apply the inverse of the rotation to
 * @return a new vector which such that u is its image by the rotation
 */
glm::dvec3
Rotation::applyInverseTo(glm::dvec3 u)
{

  double x = u.x;
  double y = u.y;
  double z = u.z;

  double s = q1 * x + q2 * y + q3 * z;
  double m0 = -q0;

  return glm::dvec3(2 * (m0 * (x * m0 - (q2 * z - q3 * y)) + s * q1) - x,
                    2 * (m0 * (y * m0 - (q3 * x - q1 * z)) + s * q2) - y,
                    2 * (m0 * (z * m0 - (q1 * y - q2 * x)) + s * q3) - z);
}

/** Apply the inverse of the rotation to a vector stored in an array.
 * @param in an array with three items which stores vector to rotate
 * @param out an array with three items to put result to (it can be the same
 * array as in)
 */
void
Rotation::applyInverseTo(double* in, double* out)
{

  const double x = in[0];
  const double y = in[1];
  const double z = in[2];

  const double s = q1 * x + q2 * y + q3 * z;
  const double m0 = -q0;

  out[0] = 2 * (m0 * (x * m0 - (q2 * z - q3 * y)) + s * q1) - x;
  out[1] = 2 * (m0 * (y * m0 - (q3 * x - q1 * z)) + s * q2) - y;
  out[2] = 2 * (m0 * (z * m0 - (q1 * y - q2 * x)) + s * q3) - z;
}

/** Apply the instance to another rotation.
 * Applying the instance to a rotation is computing the composition
 * in an order compliant with the following rule : let u be any
 * vector and v its image by r (i.e. r.applyTo(u) = v), let w be the image
 * of v by the instance (i.e. applyTo(v) = w), then w = comp.applyTo(u),
 * where comp = applyTo(r).
 * @param r rotation to apply the rotation to
 * @return a new rotation which is the composition of r by the instance
 */
Rotation
Rotation::applyTo(Rotation const& r) const
{
  return Rotation(r.q0 * q0 - (r.q1 * q1 + r.q2 * q2 + r.q3 * q3),
                  r.q1 * q0 + r.q0 * q1 + (r.q2 * q3 - r.q3 * q2),
                  r.q2 * q0 + r.q0 * q2 + (r.q3 * q1 - r.q1 * q3),
                  r.q3 * q0 + r.q0 * q3 + (r.q1 * q2 - r.q2 * q1),
                  false);
}

/** Apply the inverse of the instance to another rotation.
 * Applying the inverse of the instance to a rotation is computing
 * the composition in an order compliant with the following rule :
 * let u be any vector and v its image by r (i.e. r.applyTo(u) = v),
 * let w be the inverse image of v by the instance
 * (i.e. applyInverseTo(v) = w), then w = comp.applyTo(u), where
 * comp = applyInverseTo(r).
 * @param r rotation to apply the rotation to
 * @return a new rotation which is the composition of r by the inverse
 * of the instance
 */
Rotation
Rotation::applyInverseTo(Rotation r)
{
  return Rotation(-r.q0 * q0 - (r.q1 * q1 + r.q2 * q2 + r.q3 * q3),
                  -r.q1 * q0 + r.q0 * q1 + (r.q2 * q3 - r.q3 * q2),
                  -r.q2 * q0 + r.q0 * q2 + (r.q3 * q1 - r.q1 * q3),
                  -r.q3 * q0 + r.q0 * q3 + (r.q1 * q2 - r.q2 * q1),
                  false);
}

void
Rotation::getAngles(RotationOrder const* order,
                    double& roll,
                    double& pitch,
                    double& yaw)
{
  if (order == &RotationOrder::XYZ) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_K);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_I);
    if (v2.z < ALMOST_MINUS_1 || v2.z > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles XYZ");
    }
    roll = std::atan2(-v1.y, v1.z);
    pitch = std::asin(v2.z);
    yaw = std::atan2(-v2.y, v2.x);
  } else if (order == &RotationOrder::XZY) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_J);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_I);
    if (v2.y < ALMOST_MINUS_1 || v2.y > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles XZY");
    }
    roll = std::atan2(v1.z, v1.y);
    pitch = -std::asin(v2.y);
    yaw = std::atan2(v2.z, v2.x);
  } else if (order == &RotationOrder::YXZ) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_K);
    glm::dvec3 v2 = applyTo(RotationOrder::PLUS_J);
    if (v2.z < ALMOST_MINUS_1 || v2.z > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles YXZ");
    }
    roll = std::atan2(v1.x, v1.z);
    pitch = -std::asin(v2.z);
    yaw = std::atan2(v2.x, v2.y);
  } else if (order == &RotationOrder::YZX) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_I);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_J);
    if (v2.x < ALMOST_MINUS_1 || v2.x > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles YZX");
    }
    roll = std::atan2(-v1.z, v1.x);
    pitch = std::asin(v2.x);
    yaw = std::atan2(-v2.z, v2.y);
  } else if (order == &RotationOrder::ZXY) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_J);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_K);
    if (v2.y < ALMOST_MINUS_1 || v2.y > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles ZXY");
    }
    roll = std::atan2(-v1.x, v1.y);
    pitch = std::asin(v2.y);
    yaw = std::atan2(-v2.x, v2.z);
  } else if (order == &RotationOrder::ZYX) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_I);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_K);
    if (v2.x < ALMOST_MINUS_1 || v2.x > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles ZYX");
    }
    roll = std::atan2(v1.y, v1.x);
    pitch = -std::asin(v2.x);
    yaw = std::atan2(v2.y, v2.z);
  } else if (order == &RotationOrder::XYX) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_I);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_I);
    if (v2.x < ALMOST_MINUS_1 || v2.x > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles XYX");
    }
    roll = std::atan2(v1.y, -v1.z);
    pitch = std::acos(v2.x);
    yaw = std::atan2(v2.y, v2.z);
  } else if (order == &RotationOrder::XZX) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_I);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_I);
    if (v2.x < ALMOST_MINUS_1 || v2.x > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles XZX");
    }
    roll = std::atan2(v1.z, v1.y);
    pitch = std::acos(v2.x);
    yaw = std::atan2(v2.z, -v2.y);
  } else if (order == &RotationOrder::YXY) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_J);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_J);
    if (v2.y < ALMOST_MINUS_1 || v2.y > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles YXY");
    }
    roll = std::atan2(v1.x, v1.z);
    pitch = std::acos(v2.y);
    yaw = std::atan2(v2.x, -v2.z);
  } else if (order == &RotationOrder::YZY) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_J);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_J);
    if (v2.y < ALMOST_MINUS_1 || v2.y > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles YZY");
    }
    roll = std::atan2(v1.z, -v1.x);
    pitch = std::acos(v2.y);
    yaw = std::atan2(v2.z, v2.x);
  } else if (order == &RotationOrder::ZXZ) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_K);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_K);
    if (v2.z < ALMOST_MINUS_1 || v2.y > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles ZXZ");
    }
    roll = std::atan2(v1.x, -v1.y);
    pitch = std::acos(v2.z);
    yaw = std::atan2(v2.x, v2.y);
  } else if (order == &RotationOrder::ZYZ) {
    glm::dvec3 v1 = applyTo(RotationOrder::PLUS_K);
    glm::dvec3 v2 = applyInverseTo(RotationOrder::PLUS_K);
    if (v2.z < ALMOST_MINUS_1 || v2.z > ALMOST_PLUS_1) {
      throw HeliosException("Gimbal lock on Rotation::getAngles ZYZ");
    }
    roll = std::atan2(v1.y, v1.x);
    pitch = std::acos(v2.z);
    yaw = std::atan2(v2.y, -v2.x);
  } else {
    throw HeliosException("Unexpected RotationOrder on Rotation::getAngles");
  }
}

/** Perfect orthogonality on a 3X3 matrix.
 * @param m initial matrix (not exactly orthogonal)
 * @param threshold convergence threshold for the iterative
 * orthogonality correction (convergence is reached when the
 * difference between two steps of the Frobenius norm of the
 * correction is below this threshold)
 * @return an orthogonal matrix close to m
 * @exception NotARotationMatrixException if the matrix cannot be
 * orthogonalized with the given threshold after 10 iterations
 */
/** Compute the <i>distance</i> between two rotations.
 * <p>The <i>distance</i> is intended here as a way to check if two
 * rotations are almost similar (i.e. they transform vectors the same way)
 * or very different. It is mathematically defined as the angle of
 * the rotation r that prepended to one of the rotations gives the other
 * one:</p>
 * <pre>
 *        r<sub>1</sub>(r) = r<sub>2</sub>
 * </pre>
 * <p>This distance is an angle between 0 and &pi;. Its value is the smallest
 * possible upper bound of the angle in radians between r<sub>1</sub>(v)
 * and r<sub>2</sub>(v) for all possible vectors v. This upper bound is
 * reached for some v. The distance is equal to 0 if and only if the two
 * rotations are identical.</p>
 * <p>Comparing two rotations should always be done using this value rather
 * than for example comparing the components of the quaternions. It is much
 * more stable, and has a geometric meaning. Also comparing quaternions
 * components is error prone since for example quaternions (0.36, 0.48, -0.48,
 * -0.64) and (-0.36, -0.48, 0.48, 0.64) represent exactly the same rotation
 * despite their components are different (they are exact opposites).</p>
 * @param r1 first rotation
 * @param r2 second rotation
 * @return <i>distance</i> between r1 and r2
 */
std::ostream&
operator<<(std::ostream& out, Rotation& r)
{
  out << "[" << r.getQ0() << ", " << r.getQ1() << ", " << r.getQ2() << ", "
      << r.getQ3() << "]";
  return out;
}
