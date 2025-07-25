#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

/**
 * \f[
 *  \frac{\pi}{2}
 * \f]
 */
const double PI_HALF = M_PI / 2.0;

/**
 * \f[
 *  \frac{\pi}{4}
 * \f]
 */
const double PI_QUARTER = M_PI / 4.0;
/**
 * \frac[
 *  \frac{\pi}{8}
 * \f]
 */
const double PI_EIGHTH = M_PI / 8.0;

/**
 * \f[
 *  \frac{3{\pi}}{2}
 * \f]
 */
const double PI_3_HALF = PI_HALF * 3.0;

/**
 * \f[
 *  2{\pi}
 * \f]
 */
const double PI_2 = M_PI * 2.0;

/**
 * \f[
 *  4{\pi}
 * \f]
 */
const double PI_4 = M_PI * 4.0;
/**
 * \f[
 *  \pi^2
 * \f]
 */
const double PI_SQUARED = M_PI * M_PI;

/**
 * \f[
 *  2 \pi^2
 * \f]
 */
const double PI_SQUARED_2 = 2 * M_PI * M_PI;

/**
 * \f[
 *  \frac{180}{\pi}
 * \f]
 */
const double _180_OVER_PI = 180.0 / M_PI;

/**
 * \f[
 *  \frac{\pi}{180}
 * \f]
 */
const double PI_OVER_180 = M_PI / 180.0;

/**
 * @brief Constant representing a very close from above to -1 number
 */
const double ALMOST_MINUS_1 = -0.9999999999;
/**
 * @brief Constant representing a very close from below to 1 number
 */
const double ALMOST_PLUS_1 = 0.9999999999;

/**
 * \f[
 *  \sqrt{2}
 * \f]
 */

const double SQRT2 = 1.4142135623730951;

/**
 * @brief Speed of light in meters per second
 */
const double SPEEDofLIGHT_mPerSec = 299792458;

/**
 * @brief Speed of light in meters per nanosecond
 */
const double SPEEDofLIGHT_mPerNanosec = 0.299792458;

/**
 * @brief Speed of light in meters per picosecond
 */
const double SPEEDofLIGHT_mPerPicosec = 0.000299792458;
