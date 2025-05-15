#ifndef _SURFACEINSPECTOR_MATHS_STATISTICS_HPP_
#define _SURFACEINSPECTOR_MATHS_STATISTICS_HPP_

#include <vector>

#include <armadillo>

#include <surfaceinspector/util/Object.hpp>
#include <surfaceinspector/util/draggers/IDragger.hpp>

using std::vector;

using SurfaceInspector::util::Object;
using SurfaceInspector::util::draggers::IDragger;

namespace SurfaceInspector {
namespace maths {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing common statistics operations
 */
class Statistics : public Object
{
public:
  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Compute the mean of given values
   * @tparam T Type of numerical variable
   * @param vals Values to compute its mean
   * @return Mean of given values
   */
  template<typename T>
  static T mean(vector<T> const& vals);
  /**
   * @brief Compute the variance of given values, which requires to
   *  previously compute the mean
   * @tparam T Type of numerical variable
   * @param vals Values to compute its variance
   * @return Variance of given values
   */
  template<typename T>
  static T variance(vector<T> const& vals);
  /**
   * @brief Compute the variance of given values, considering provided mean
   * @tparam T Type of numerical variable
   * @param vals Values to compute its variance
   * @param mean Mean of values
   * @return Variance of given values
   */
  template<typename T>
  static T variance(vector<T> const& vals, T const mean);
  /**
   * @brief Compute the standard deviation of given values, which requires to
   *  previously compute the variance
   * @tparam T Type of numerical variable
   * @param vals Values to compute its standard deviation
   * @return Standard deviation of given values
   */
  template<typename T>
  static T stdev(vector<T> const& vals);
  /**
   * @brief Compute the standard deviation of given values, considering
   *  provided mean
   * @tparam T Type of numerical variable
   * @param vals Values to compute its standard deviation
   * @param mean Mean of values
   * @return Standard deviation of given values
   */
  template<typename T>
  static T stdev(vector<T> const& vals, T const mean);
  /**
   * @brief Compute the standard deviation for given variance
   * @tparam T Type of numerical variable
   * @param variance Variance used to compute standard deviation
   * @return Standard deviation of given variance
   */
  template<typename T>
  static inline T stdev(T const variance)
  {
    return std::sqrt(variance);
  };
  /**
   * @brief Compute the covariance between X and Y
   *
   * \f[
   *  \sigma_{x,y} = \frac{1}{n}
   *      \sum_{i=1}^{n}{(x_{i} - \mu_{x})(y_{i} - \mu_{y})}
   * \f]
   *
   * @tparam T Type of numerical variable
   * @param X X values to compute its covariance with respect to Y
   * @param Y Y values to compute its covariance with respect to X
   * @param besselCorrection If true, then bessel correction will be applied
   *  and covariance will be computed considering (n-1) instead of n
   * @return Covariance between X and Y
   */
  template<typename T>
  static T covariance(vector<T> const& X,
                      vector<T> const& Y,
                      bool besselCorrection = false);

  /**
   * @brief Compute the covariance between X and Y, considering provided
   *  means
   *
   * \f[
   *  \sigma_{x,y} = \frac{1}{n}
   *      \sum_{i=1}^{n}{(x_{i} - \mu_{x})(y_{i} - \mu_{y})}
   * \f]
   *
   * @tparam T Type of numerical variable
   * @param X X values to compute its covariance with respect to Y
   * @param Y Y values to compute its covariance with respect to X
   * @param xMean Mean of X
   * @param yMean Mean of Y
   * @param besselCorrection If true, then bessel correction will be applied
   *  and covariance will be computed considering (n-1) instead of n
   * @return Covariance between X and Y
   */
  template<typename T>
  static T covariance(vector<T> const& X,
                      vector<T> const& Y,
                      T xMean,
                      T yMean,
                      bool besselCorrection = false);
  /**
   * @brief Compute matrix of covariances
   * @tparam T Type of numerical variable
   * @param dataset Dataset to generate matrix of covariances from.
   *  It must be organized so each row (dataset[row]) contains all values for
   *  the same variable.
   *  For instance, let D be a dataset such that:
   *  \f[
   *      D = \left( \begin{array}
   *          .x_{1} & x_{2} & x_{3} & x_{4} & x_{5} \\
   *          y_{1} & y_{2} & y_{3} & y_{4} & y_{5} \\
   *          z_{1} & z_{2} & z_{3} & z_{4} & z_{5}
   *      \end{array} \right)
   *  \f]
   *  So row[0] would contain \f$\{x_{1},x_{2},x_{3},x_{4},x_{5}\}\f$,
   *  row[1] would contain \f$\{y_{1}, y_{2}, y_{3}, y_{4}, y_{5}\}\f$ and
   *  row[2] would contain \f$\{z_{1}, z_{2}, z_{3}, z_{4}, z_{5}\}\f$
   * @param besselCorrection If true, then bessel correction will be applied
   *  and covariance will be computed considering (n-1) instead of n
   * @return Covariance matrix
   */
  template<typename T>
  static arma::Mat<T> covarianceMatrix(vector<vector<T>> dataset,
                                       bool besselCorrection = false);
  /**
   * @brief Compute matrix of covariances
   * @tparam T Type of numerical variable
   * @param dataset Dataset to generate matrix of covariances from.
   *  It must be organized so each row (dataset[row]) contains all values for
   *  the same variable.
   *  For instance, let D be a dataset such that:
   *  \f[
   *      D = \left( \begin{array}
   *          .x_{1} & x_{2} & x_{3} & x_{4} & x_{5} \\
   *          y_{1} & y_{2} & y_{3} & y_{4} & y_{5} \\
   *          z_{1} & z_{2} & z_{3} & z_{4} & z_{5}
   *      \end{array} \right)
   *  \f]
   *  So row[0] would contain \f$\{x_{1},x_{2},x_{3},x_{4},x_{5}\}\f$,
   *  row[1] would contain \f$\{y_{1}, y_{2}, y_{3}, y_{4}, y_{5}\}\f$ and
   *  row[2] would contain \f$\{z_{1}, z_{2}, z_{3}, z_{4}, z_{5}\}\f$
   * @param means The mean for each variable so means[i] would contain the
   *  mean for all values at row dataset[i]
   * @param besselCorrection If true, then bessel correction will be applied
   *  and covariance will be computed considering (n-1) instead of n
   * @return Covariance matrix
   */
  template<typename T>
  static arma::Mat<T> covarianceMatrix(vector<vector<T>> dataset,
                                       vector<T> means,
                                       bool besselCorrection = false);

  /**
   * @brief Compute quantiles
   * @tparam T Type of numerical variable
   * @param data Vector containing data to generate quantiles from
   * @param nQuantiles Number of quantiles to obtain. For instance, 3 would
   *  imply obtaining the 3 quartiles, while 99 would lead to obtain the 99
   *  percentiles
   * @return Vector containing obtained quantiles
   */
  template<typename T>
  static vector<T> quantiles(arma::Col<T> data, size_t nQuantiles = 3);

  /**
   * @brief Like quantiles(armas::vec, size_t) function but receiving data as
   *  a standard vector instead of an armadillo vector
   * @see Statistics::quantiles(arma::vec, size_t)
   */
  template<typename T>
  static vector<T> quantiles(vector<T> const& data, size_t nQuantiles = 3);

  /**
   * @brief Remove those values which are considered outliers
   *
   * Let \f$\overline{V}\f$ be the vector of values, \f$k\f$ be the filtering
   *  factor and assume \f$\Delta = Q_{n}-Q_{1}\f$ as the difference between
   *  last and first quantile. When 3 quantiles are used, quartiles are used
   *  hence \f$\Delta = Q_{3} - Q_{1}\f$ which would be the
   *  classical/standard quantile filtering method.
   * Below expression defines when a value from \f$\overline{V}\f$ will also
   *  be in filtered vector \f$\overline{F}\f$:
   * \f[
   *   \forall v \in \overline{V} ,\, v \in [Q_{1}-k\Delta, Q_{n}+k\Delta]
   *   \Rightarrow v \in \overline{F}
   * \f]
   *
   * @tparam T Numerical type for values vector
   * @param values Vector of values to be filtered
   * @param filterFactor Factor defining filtering boundaries
   * @return Amount of filtered values
   */
  template<typename T>
  static size_t quantileFiltering(vector<T>& values,
                                  size_t nQuantiles = 3,
                                  double filterFactor = 1.5);

  /**
   * @brief Obtain the trustable minimum between all elements in given
   *  collection \f$S\f$
   *
   * Let \f$S\f$ be a collection of elements of the same type.
   *
   * Now, let \f$Z = \left\{z_1, \ldots, z_m \right\}\f$ be the collection of
   *  \f$m\f$ lowest elements in ascending order.
   *
   * From \f$Z\f$, it is possible to define the collection
   *  \f$\Delta = \left\{\Delta_1, \ldots, \Delta_{m-1} :
   *      \Delta_i = z_{i+1} - z_{i}\right\}\f$
   *
   * For each \f$\Delta_i \in \Delta\f$, if \f$\Delta_i > \tau\f$ then
   *  remove \f$R=\{z_j : j \leq i\}\f$ from \f$Z\f$, while adding
   *  \f$A=\{z_j : i < j \leq i+|R|\}\f$ (where \f$|R|\f$ is the number of
   *  removed elements).
   * This means the updated \f$Z\f$ collection can be defined as
   *  \f$Z' = Z \setminus R + A = \{z_1', \ldots, z_m'\}\f$.
   * In consequence, the updated \f$\Delta\f$ collection can be expressed
   *  as \f$\Delta' = \{\Delta_1', \ldots, \Delta_{m-1}' :
   *  \Delta_i' = z_{i+1}' - z_i'\}\f$.
   *
   * Aforementioned process must be repeated until \f$|\Delta| = m-1\f$ and
   *  \f$\forall  \Delta_i \in \Delta,\, \Delta_{i} \leq \tau\f$ (where
   *  \f$|\Delta|\f$ is the number of elements in \f$\Delta\f$ collection).
   * Whenever this cannot be satisfied, the trustable minimum is considered
   *  to be undefined and \f$g=0\f$ by default.
   * Otherwise, the trustable minimum is defined as \f$g=\min(Z)\f$ where
   *  \f$Z\f$ is the collection of trustable lowest elements obtained
   *  through previously defined iterative process.
   *
   * <p><b>PARTICULAR CASE</b> \f$m=1\f$</p>
   * If \f$m=1\f$ then the trustable minimum of the collection will be the
   *  minimum of the collection itself.
   * This happens because \f$m=1\f$ implies no checks will be done.
   *
   *
   * @tparam T Numerical type for \f$S\f$ collection elements
   * @param[in] S Collection \f$S\f$ of elements implemented as vector
   * @param[in] m Number of \f$m\f$ values involved in trustability analysis
   * @param[in] tau Differential threshold \f$\tau\f$ defining trustability
   * @param[out] g Minimum trustable element in \f$S\f$
   * @param[out] discardsCount If it is a not null pointer, then number of
   *  checks leading to a discard is stored here
   * @return True if trustable minimum was found (defined), false otherwise
   *  (undefined)
   * @see SurfaceInspector::maths::Statistics::findTrustableMax
   */
  template<typename T>
  static bool findTrustableMin(vector<T> const& S,
                               size_t const m,
                               T const tau,
                               T& g,
                               size_t* discardsCount = nullptr);
  /**
   * @brief Like findTrustableMin but with maximum value instead of minimum
   * @see SurfaceInspector::maths::Statistics::findTrustableMin
   */
  template<typename T>
  static bool findTrustableMax(vector<T> const& S,
                               size_t const m,
                               T const tau,
                               T& g,
                               size_t* discardsCount = nullptr);

  // ***  INNER METHODS  *** //
  // *********************** //
protected:
  /**
   * @brief Common mechanics for find trustable methods
   * @see SurfaceInspector::maths::Statistics::findTrustableMin
   * @see SurfaceInspector::maths::Statistics::findTrustableMax
   */
  template<typename T>
  static bool _findTrustable(vector<T> const& S,
                             size_t const m,
                             T const tau,
                             IDragger<double, vector<double>>& drg,
                             T& g,
                             size_t* discardsCount = nullptr);
};

}
}

#include <surfaceinspector/maths/Statistics.tpp>

#endif
