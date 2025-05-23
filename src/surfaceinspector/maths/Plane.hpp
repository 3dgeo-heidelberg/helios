#pragma once

#include <vector>

#include <armadillo>

#include <surfaceinspector/util/Object.hpp>
namespace SurfaceInspector {
namespace maths {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a plane
 * @tparam T Type of number
 */
template<typename T>
class Plane : public SurfaceInspector::util::Object
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Coordinates for plane centroid
   */
  std::vector<T> centroid;
  /**
   * @brief Plane orthonormal vector
   */
  std::vector<T> orthonormal;
  /**
   * @brief When the plane has been obtained through fitting to a sample,
   *  scatter will quantify variation with respect to elements in the sample
   * @see PlaneFitter
   */
  T scatter;
  /**
   * @brief When the plane has been obtained through fitting to a sample,
   *  curvature is defined by expression:
   *
   * \f[
   *  \frac{\lambda_{1}}{\sum_{i}^{n}\lambda_{i}}
   * \f]
   *
   * Where \f$\lambda_{i}\f$ is the ith eigen value or ith singular value,
   *  depending on used method. So \f$\lambda_{1}\f$ will be the eigen or
   *  singular value for the plane which maximizes distance with respect
   *  to sample set while \f$lambda_{n}\f$ will be the one which minimizes it
   *
   * @see PlaneFitter
   */
  T curvature;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default plane constructor
   */
  Plane() = default;
  /**
   * @brief Build a plane with given centroid, orthonormal and scatter
   * @param centroid Centroid for the plane
   * @param orthonormal Orthonormal for the plane
   * @param scatter Scatter for the plane
   * @see Plane:centroid
   * @see Plane::orthonormal
   * @see Plane::scatter
   * @see Plane::curvature
   */
  Plane(std::vector<T> centroid,
        std::vector<T> orthonormal,
        T scatter = 0,
        T curvature = 0)
    : centroid(centroid)
    , orthonormal(orthonormal)
    , scatter(scatter)
    , curvature(curvature) {};
  virtual ~Plane() {};
};

}
}
