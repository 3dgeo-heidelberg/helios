#pragma once

#include <MathConstants.h>

class MathConverter{
public:
    // ***  CONVERSION METHODS  *** //
    // **************************** //
    /**
     * @brief Receive radians, return degrees
     */
    static inline double radiansToDegrees(double radians)
    {return radians * _180_OVER_PI;}
    /**
     * @brief Receive degres, return radians
     */
    static inline double degreesToRadians(double degrees)
    {return degrees * PI_OVER_180;}
};