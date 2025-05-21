#ifndef _SURFACEINSPECTOR_MATHS_DISTANCES_HPP_
#define _SURFACEINSPECTOR_MATHS_DISTANCES_HPP_

#include <surfaceinspector/util/Object.hpp>
#include <vector>

using std::vector;
using SurfaceInspector::util::Object;

namespace SurfaceInspector {
namespace maths {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing common distance computations
 */
class Distances : public Object
{
public:
  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Compute the manhattan distance between p and q
   *
   * \f[
   *  \sum_{i=1}^{n}{\left| p_{i} - q_{i} \right| }
   * \f]
   *
   * @tparam T Type of numerical variable
   * @param p Object which manhattan distance with respect to q must be
   *  calculated
   * @param q Object which manhattan distance with respect to p must be
   *  calculated
   */
  template<typename T>
  static T manhattan(vector<T> const& p, vector<T> const& q);
  /**
   * @brief Compute the euclidean distance between p and q
   *
   * \f[
   *  \sqrt{\sum_{i=1}^{n}{\left( p_{i} - q_{i} \right)^{2}}}
   * \f]
   *
   * @tparam T Type of numerical variable
   * @param p Object which euclidean distance with respect to q must be
   *  calculated
   * @param q Object which euclidean distance with respect to p must be
   *  calculated
   */
  template<typename T>
  static T euclidean(vector<T> const& p, vector<T> const& q);
  /**
   * @see Distances::euclidean(vector<T> const&, vector<T> const &)
   */
  template<typename T>
  static T euclidean(T const px, T const py, T const qx, T const qy);
  /**
   * @see Distances::euclidean(vector<T> const&, vector<T> const &)
   */
  template<typename T>
  static T euclidean(T const px,
                     T const py,
                     T const pz,
                     T const qx,
                     T const qy,
                     T const qz);
  /**
   * @brief Compute the minkowski distance between p and q
   *
   * \f[
   *  \left(\sum_{i=1}^{n}{\left|p_{i}-q_{i}\right|^{d}}\right)^{\frac{1}{d}}
   * \f]
   *
   * @tparam T Type of numerical variable
   * @param p Object which euclidean distance with respect to q must be
   *  calculated
   * @param q Object which euclidean distance with respect to p must be
   *  calculated
   */
  template<typename T>
  static T minkowski(int d, vector<T> const& p, vector<T> const& q);
};
}
}

#include <maths/Distances.tpp>

#endif
