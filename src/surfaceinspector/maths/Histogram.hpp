#pragma once

#include <vector>

#include <surfaceinspector/maths/functions/GaussianFunction.hpp>
#include <surfaceinspector/util/Object.hpp>

namespace SurfaceInspector {
namespace maths {

/**
 * @author Alberto M. Esmoris PEna
 * @version 1.0
 *
 * @tparam T Type of element
 * @brief Class for representation and handling of 1D histograms
 */
template<typename T>
class Histogram : public SurfaceInspector::util::Object
{
private:
  // *********************** //

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The number of elements considered to build the histogram
   */
  std::size_t m;
  /**
   * @brief The number of bins
   */
  std::size_t n;

  /**
   * @brief The minimum value on data used to build the histogram
   */
  T xmin;
  /**
   * @brief The maximum value on data used to build the histogram
   */
  T xmax;
  /**
   * @brief The difference between maximum and minimum value
   */
  T delta;
  /**
   * @brief The step between bins. It can also be understood as the bin size
   */
  T step;
  /**
   * @brief The norm for the unitary area histogram
   * @see Histogram::computeDensity
   */
  T norm;

  /**
   * @brief The absolute frequency, it is number of elements in each bin
   *
   * \f$c_i = k\f$ means there are k elements in the i-th bin
   */
  std::vector<std::size_t> c;
  /**
   * @brief The relative frequency for each bin
   */
  std::vector<double> r;
  /**
   * @brief The density for each bin corresponding to the unitary area
   *  version of the histogram
   * @see Histogram::computeDensity
   */
  std::vector<double> d;

  /**
   * @brief The start value for each bin
   *
   * \f$a_i = x\f$ means x is the start value for i-th bin
   */
  std::vector<T> a;
  /**
   * @brief The end value for each bin
   *
   * \f$b_i = x\f$ means y is the end value for i-th bin
   */
  std::vector<T> b;

  // *** CONSTRUCTION / DESTRUCTION  *** //
  // *********************************** //
  /**
   * @brief Build a histogram from given vector of values \f$\vec{x}\f$ and
   *  requested number of bins \f$n\f$
   * @param x The vector of values
   * @param n Requested number of bins
   * @param relative Compute the relative frequencies if true. Skip their
   *  computation if false
   * @param density Compute the density if true. Skip its computation if
   *  false
   */
  Histogram(std::vector<T> x,
            std::size_t n = 256,
            bool relative = true,
            bool density = true);
  /**
   * @brief Build a histogram starting at xmin and ending at xmax populated
   *  by given vector of values \f$\vec{x}\f$ and with requested number of
   *  bins \f$n\f$
   * @param xmin Where the histogram must start
   * @param xmax Where the histogram must end
   * @param x The vector of values
   * @param n Requested number of bins
   * @param relative Compute the relative frequencies if true. Skip their
   *  computation if false
   * @param density Compute the density if true. Skip its computation if
   *  false
   */
  Histogram(T xmin,
            T xmax,
            std::vector<T> x,
            std::size_t n = 256,
            bool relative = true,
            bool density = true);
  /**
   * @brief Virtual destructor for the histogram
   */
  virtual ~Histogram() = default;

  // ***  HISTOGRAM METHODS  *** //
  // *************************** //
  /**
   * @brief Estimate a gaussian function from the histogram. <b>Notice</b>
   *  this method MUST NOT be called if histogram's density is not
   *  available.
   * @see SurfaceInspector::maths::functions::GaussianFunction
   */
  SurfaceInspector::maths::functions::GaussianFunction<T> estimateGaussian();
  /**
   * @brief Obtain the cut point (value) \f$\tau\f$ so approximately
   *  \f$100p \%\f$ of the elements are greater than it.
   *
   * <b>Notice</b> this method MUST NOT be called if histogram's relative
   *  frequencies are not available.
   *
   * @param p Percentage in \f$\left[0, 1\right]\f$ interval
   * @return Cut point \f$\tau\f$
   */
  T findCutPoint(double p);
  /**
   * @brief Compute the cumulative sum of absolute frequencies inside given
   *  index interval
   *
   * \f[
   *  \sum_{i=\alpha}^{\beta-1}{c_i}
   * \f]
   *
   * @param start The inclusive start index of absolute cumsum \f$\alpha\f$
   * @param end The exclusive end index of absolute cumsum \f$\beta\f$
   * @return Absoulte cumsum in index interval \f$[\alpha, \beta)\f$
   */
  std::size_t absCumsum(std::size_t const start, std::size_t const end);
  /**
   * @brief Like absCumsum(size_t const, size_t const) but for the entire
   *  histogram
   * @see Histogram::absCumsum(size_t const, size_t const)
   */
  inline T absCumsum() { return absCumsum(0, c.size()); }

private:
  // ***  INNER METHODS  *** //
  // *********************** //
  /**
   * @brief Extract min and max values from vector of values
   * @param x The vector of values
   */
  void extractMinMax(std::vector<T> const& x);

  /**
   * @brief Compute the \f$[a, b]\f$ interval for each bin
   */
  void computeBinningIntervals();
  /**
   * @brief Count the number of elements in each bin, considering given
   *  vector of values \f$\vec{x}\f$
   *
   * Let \f$\Delta = max\left(\vec{x}\right) - min\left(\vec{x}\right)\f$ so:
   * \f[
   *  \forall x \in \vec{x},
   *      \hat{x} = \frac{x-min\left(\vec{x}\right)}{\Delta}
   * \f]
   *
   * Now for any \f$i\f$ bin, its count \f$c_i\f$ can be defined as follows:
   * \f[
   *  c_i = \left|\left\{
   *      \hat{x} : \left\lfloor n\hat{x}\right\rfloor = i
   *  \right\}\right|
   * \f]
   *
   * @param x the Vector of values
   */
  void recount(std::vector<T> const& x);
  /**
   * @brief Compute the relative frequencies for each bin
   *
   * Let \f$m\f$ be the number of considered data points when building the
   *  histogram and \f$c_i\f$ the count of elements at i-th bin. Thus,
   *  the relative frequency for i-th bin \f$r_i\f$ is:
   * \f[
   *  r_i = \frac{c_i}{m}
   * \f]
   */
  void computeRelativeFrequencies();

  /**
   * @brief Compute the density (or normalized) histogram.
   *
   * Let \f$h\f$ be the step or bin size, and \f$c_i\f$ the count for i-th
   *  bin. Now the norm \f$\lambda\f$ can be defined as:
   * \f[
   *  \lambda = \sum_{i=1}^{n} h c_i
   * \f]
   *
   * Then, the unitary area density (AKA normalized frequency) for each
   *  i-th bin \f$d_i\f$ can be calculated.
   * \f[
   *  d_i = \frac{c_i}{\lambda}
   * \f]
   *
   * @see Histogram::norm
   * @see Histogram::d
   */
  void computeDensity();
};
}
}

#include <surfaceinspector/maths/Histogram.tpp>
