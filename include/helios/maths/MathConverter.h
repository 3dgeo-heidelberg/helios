#pragma once

#include <helios/maths/MathConstants.h>

class MathConverter
{
public:
  // ***  CONVERSION METHODS  *** //
  // **************************** //
  /**
   * @brief Receive radians, return degrees
   */
  static inline double radiansToDegrees(double const radians)
  {
    return radians * _180_OVER_PI;
  }
  /**
   * @brief Receive degrees, return radians
   */
  static inline double degreesToRadians(double const degrees)
  {
    return degrees * PI_OVER_180;
  }
};
