#include <surfaceinspector/maths/Vector.hpp>
#include <surfaceinspector/util/draggers/IDragger.hpp>
#include <surfaceinspector/util/draggers/MaxDragger.hpp>
#include <surfaceinspector/util/draggers/MinDragger.hpp>

using SurfaceInspector::maths::Statistics;
using SurfaceInspector::maths::Vector;
using SurfaceInspector::util::draggers::IDragger;
using SurfaceInspector::util::draggers::MaxDragger;
using SurfaceInspector::util::draggers::MinDragger;

// ***  STATIC METHODS  *** //
// ************************ //
template<typename T>
T
SurfaceInspector::maths::Statistics::mean(std::vector<T> const& vals)
{
  T mean = 0;
  for (T const& val : vals)
    mean += val;
  return mean / ((T)vals.size());
}
template<typename T>
T
SurfaceInspector::maths::Statistics::variance(std::vector<T> const& vals)
{
  T mean = Statistics::mean(vals);
  return Statistics::variance(vals, mean);
}
template<typename T>
T
SurfaceInspector::maths::Statistics::variance(std::vector<T> const& vals,
                                              T const mean)
{
  T variance = 0;
  for (T const& val : vals) {
    T diff = mean - val;
    variance += diff * diff;
  }
  return variance / ((T)vals.size());
}
template<typename T>
T
SurfaceInspector::maths::Statistics::stdev(std::vector<T> const& vals)
{
  T variance = SurfaceInspector::maths::Statistics::variance(vals);
  return SurfaceInspector::maths::Statistics::stdev(variance);
}
template<typename T>
T
SurfaceInspector::maths::Statistics::stdev(std::vector<T> const& vals,
                                           T const mean)
{
  T variance = SurfaceInspector::maths::Statistics::variance(vals, mean);
  return SurfaceInspector::maths::Statistics::stdev(variance);
}
template<typename T>
T
SurfaceInspector::maths::Statistics::covariance(std::vector<T> const& X,
                                                std::vector<T> const& Y,
                                                bool besselCorrection)
{
  return covariance(X, Y, mean(X), mean(Y), besselCorrection);
}
template<typename T>
T
SurfaceInspector::maths::Statistics::covariance(std::vector<T> const& X,
                                                std::vector<T> const& Y,
                                                T xMean,
                                                T yMean,
                                                bool besselCorrection)
{
  T covar = 0.0;
  std::size_t n = X.size();
  double N = (double)n;
  if (besselCorrection)
    N = (double)(n - 1);
  for (std::size_t i = 0; i < n; i++) {
    covar += (X[i] - xMean) * (Y[i] - yMean);
  }
  return covar / N;
}
template<typename T>
arma::Mat<T>
SurfaceInspector::maths::Statistics::covarianceMatrix(
  std::vector<std::vector<T>> dataset,
  bool besselCorrection)
{
  // Compute means
  std::vector<T> means;
  for (std::vector<T>& vals : dataset)
    means.push_back(SurfaceInspector::maths::Statistics::mean(vals));

  // Obtain covariance matrix
  return covarianceMatrix(dataset, means, besselCorrection);
}
template<typename T>
arma::Mat<T>
SurfaceInspector::maths::Statistics::covarianceMatrix(
  std::vector<std::vector<T>> dataset,
  std::vector<T> means,
  bool besselCorrection)
{
  // Compute covariance matrix
  std::size_t n = dataset.size();
  arma::Mat<T> covars(n, n);
  for (std::size_t i = 0; i < n; i++) {
    for (std::size_t j = 0; j < i; j++) {
      covars[i * n + j] = covars[i + j * n];
    }
    for (std::size_t j = i; j < n; j++) {
      covars[i * n + j] = SurfaceInspector::maths::Statistics::covariance(
        dataset[i], dataset[j], means[i], means[j], besselCorrection);
    }
  }

  // Return covariance matrix
  return covars;
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Statistics::quantiles(arma::Col<T> data,
                                               std::size_t nQuantiles)
{
  // Prepare quantiles computation
  arma::Col<double> quantiles(nQuantiles);
  double splits = (double)(nQuantiles + 1);
  for (std::size_t i = 1; i <= nQuantiles; i++) {
    quantiles(i - 1) = ((double)(i)) / splits;
  }

  // Compute quantiles
  arma::vec _Q = arma::quantile(data, quantiles);

  // Translate quantiles to standard vector
  std::vector<T> Q;
  for (std::size_t i = 0; i < nQuantiles; i++)
    Q.push_back(_Q[i]);

  // Return
  return Q;
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Statistics::quantiles(std::vector<T> const& data,
                                               std::size_t nQuantiles)
{
  std::size_t n = data.size();
  arma::Col<T> vec(n);
  for (std::size_t i = 0; i < n; i++)
    vec[i] = data[i];
  return quantiles(vec, nQuantiles);
}

template<typename T>
std::size_t
SurfaceInspector::maths::Statistics::quantileFiltering(std::vector<T>& values,
                                                       std::size_t nQuantiles,
                                                       double filterFactor)
{
  std::size_t filteredCount = 0;
  std::vector<T> Q = quantiles<T>(values, nQuantiles);
  T DELTA = Q[nQuantiles - 1] - Q[0];
  T minBound = Q[0] - filterFactor * DELTA;
  T maxBound = Q[nQuantiles - 1] + filterFactor * DELTA;
  for (std::size_t i = 0; i < values.size(); i++) {
    T v = values[i];
    if (v < minBound || v > maxBound) {
      values.erase(values.begin() + i);
      --i;
      ++filteredCount;
    }
  }
  return filteredCount;
}

template<typename T>
bool
SurfaceInspector::maths::Statistics::findTrustableMin(
  std::vector<T> const& S,
  std::size_t const m,
  T const tau,
  T& g,
  std::size_t* discardsCount)
{
  // Initialize discards count if not null
  if (discardsCount != nullptr)
    *discardsCount = 0;

  // Handle particular case m=1
  if (m == 1) {
    g = SurfaceInspector::maths::Vector<T>::min(S);
    return true;
  }

  // Compute general case
  SurfaceInspector::util::draggers::MinDragger<T> drg =
    SurfaceInspector::util::draggers::MinDragger<T>(
      std::vector<T>(S)); // To drag min1, min2, ...
  return _findTrustable(S, m, tau, drg, g, discardsCount);
}

template<typename T>
bool
SurfaceInspector::maths::Statistics::findTrustableMax(
  std::vector<T> const& S,
  std::size_t const m,
  T const tau,
  T& g,
  std::size_t* discardsCount)
{
  // Initialize discards count if not null
  if (discardsCount != nullptr)
    *discardsCount = 0;

  // Handle particular case m=1
  if (m == 1) {
    g = SurfaceInspector::maths::Vector<T>::max(S);
    return true;
  }

  // Compute general case
  SurfaceInspector::util::draggers::MaxDragger<T> drg =
    SurfaceInspector::util::draggers::MaxDragger<T>(
      std::vector<T>(S)); // To drag max1, max2, ...
  return _findTrustable(S, m, tau, drg, g, discardsCount);
}

// ***  INNER METHODS  *** //
// *********************** //
template<typename T>
bool
SurfaceInspector::maths::Statistics::_findTrustable(
  std::vector<T> const& S,
  std::size_t const m,
  T const tau,
  IDragger<double, std::vector<double>>& drg,
  T& g,
  std::size_t* discardsCount)
{
  // Prepare variables
  std::size_t m_1 = m - 1;   // Cardinality of Delta collection (num. elements)
  std::size_t m_2 = m_1 - 1; // m-2
  std::vector<T> Z(m);       // The Z collection
  std::vector<T> D;          // The Delta collection

  // Check there are enough elements to even try
  if (S.size() < m) {
    g = 0;
    return false;
  }

  // Compute Z initial population
  for (std::size_t i = 0; i < m; ++i)
    Z[i] = drg.next();
  D = SurfaceInspector::maths::Vector<T>::diff(
    Z); // Delta as discrete differences of Z

  // Iterative delta check
  std::size_t nDiscards = 0;
  bool check = false, valid = false;
  while (!check) { // While !check ~ While checking not finished
    check = true;  // Check is expected to finish at current iteration
    valid = true;  // Current iteration is expected to be valid
    for (std::size_t i = m_2; i >= 0; --i) { // The delta check itself
      if (std::fabs(D[i]) > tau) {           // Handle too big difference case
        // Handle logic
        valid = false;     // Current check status is false (not valid)
        nDiscards = i + 1; // How many discards/removals must be done
        if (nDiscards > S.size())
          break;       // Not enough elements to update
        check = false; // There are enough elements, do further checks

        // Update Z and D (Delta)
        for (std::size_t j = i; j < m_1; ++j)
          Z[j - i] = Z[j + 1]; // Move to the left
        for (std::size_t j = m_1 - i; j < m; ++j)
          Z[j] = drg.next(); // Add to the right
        D = SurfaceInspector::maths::Vector<T>::diff(
          Z); // Update discrete differences of Z

        // Increase discards count if not null
        if (discardsCount != nullptr)
          (*discardsCount) += nDiscards;

        break; // Next delta check outer loop iteration
      }
    }
  }

  // Handle undefined trustable case
  if (D.size() != m_1 || !valid) {
    g = 0;
    return false;
  }

  // Handle defined trustable case
  g = Z[0];
  return true;
  // --- General case : END
}
