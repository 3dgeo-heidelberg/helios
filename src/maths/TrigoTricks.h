#pragma once

#include <maths/MathConstants.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Just some trigonometric tricks to improve coding experience
 */
class TrigoTricks
{
public:
  // ***  RADIANS UTILS  *** //
  // *********************** //
  /**
   * @brief Clip given angle (in radians) to avoid \f$\sin(\varphi') = 0\f$.
   *
   * \f[
   *  \varphi' \in [0+\epsilon, \pi-\epsilon] \cup
   *      [\pi+\epsilon, 2\pi-\epsilon]
   * \f]
   * @tparam DecimalType The type of decimal number (e.g., double)
   * @param phi The variable to be clip, \f$\varphi \in [-2\pi, 2\pi]\f$
   * @param eps The threshold to clip the given angle
   * @return The clipped variable \f$\varphi'\f$
   */
  template<typename DecimalType>
  static inline DecimalType clipZeroSinRadians(DecimalType const phi,
                                               DecimalType const eps)
  {
    // Near 0
    if (phi > -eps && phi < eps) {
      if (phi < 0)
        return -eps; // -eps
      return eps;    // eps
    }
    // Near -pi
    if (phi > -M_PI - eps && phi < -M_PI + eps) {
      if (phi < -M_PI)
        return -M_PI - eps; // -pi-eps
      return -M_PI + eps;   // -pi+eps
    }
    // Near pi
    if (phi > M_PI - eps && phi < M_PI + eps) {
      if (phi < M_PI)
        return M_PI - eps; // pi-eps
      return M_PI + eps;   // pi+eps
    }
    // Near -2pi
    if (phi > -PI_2 - eps && phi < -PI_2 + eps) {
      if (phi < -PI_2)
        return -PI_2 - eps; // -pi-eps
      return -PI_2 + eps;   // -pi+eps
    }
    // Near 2pi
    if (phi > PI_2 - eps && phi < PI_2 + eps) {
      if (phi < PI_2)
        return PI_2 - eps; // pi-eps
      return PI_2 + eps;   // pi+eps
    }
    // No worries (assuming phi in [-2pi, 2pi] as input domain restriction)
    return phi;
  }
};
