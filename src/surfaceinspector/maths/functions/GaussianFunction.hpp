#pragma once

#include <cmath>

#include <surfaceinspector/maths/functions/IMathFunction.hpp>

namespace SurfaceInspector {
namespace maths {
namespace functions {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @tparam T Input/Output type
 * @brief Implementation of a gaussian function
 *
 * \f[
 *  f : \mathbb{R^{1}} \rightarrow \mathbb{R^{1}} \\
 *  f(x) = \frac{e^{-\frac{x^2}{2\sigma^{2}}}}{\sqrt{2\pi}\sigma}
 * \f]
 */
template<typename T>
class GaussianFunction
  : public SurfaceInspector::maths::functions::IMathFunction<T, T>
{
public:
  // ***  CONSTANTS  *** //
  // ******************* //
  static T const SQRT2PI = (T)2.5066282746310002;

  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief \f$\mu\f$
   */
  T mu;
  /**
   * @brief \f$\sigma\f$
   */
  T sigma;
  /**
   * @brief \f$\sigma^{2}\f$
   */
  T sigmaSquare;
  /**
   * @brief \f$2\sigma^{2}\f$
   */
  T twiceSigmaSquare;
  /**
   * @brief \f$\sqrt{2\pi} \sigma\f$
   */
  T sqrt2PiSigma;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a gaussian function
   */
  GaussianFunction(T mu, T sigma, T sigmaSquare)
    : mu(mu)
    , sigma(sigma)
    , sigmaSquare(sigmaSquare)
    , twiceSigmaSquare(2 * sigmaSquare)
    , sqrt2PiSigma(SQRT2PI * sigma)
  {
  }
  /**
   * @brief Build a gaussian function
   */
  GaussianFunction(T mu, T sigma)
    : GaussianFunction(mu, sigma, sigma * sigma)
  {
  }
  virtual ~GaussianFunction() = default;

  // ***  MATH FUNCTION INTERFACE  *** //
  // ********************************* //
  /**
   * @see Implementation of the gaussian function
   *
   * \f[
   *  f : \mathbb{R^{1}} \rightarrow \mathbb{R^{1}} \\
   *  f(x) = \frac{e^{-\frac{x^2}{2\sigma^{2}}}}{\sqrt{2\pi}\sigma}
   * \f]
   */
  T operator()(T const& x) override
  {
    return std::exp(-(x * x) / (twiceSigmaSquare)) / sqrt2PiSigma;
  }
};
}
}
}
