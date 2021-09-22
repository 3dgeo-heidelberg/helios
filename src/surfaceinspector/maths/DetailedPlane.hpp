#pragma once

#include <surfaceinspector/maths/Plane.hpp>

namespace SurfaceInspector { namespace maths{

template <typename T>
class DetailedPlane : public Plane<T>{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Sum of eigenvalues or singular values coming from plane fitting
     *  process
     *
     * \f[
     *  \sum_{i=1}^{n}{\lambda_{i}}
     * \f]
     */
    T sum;

    /**
     * @brief Omnivariance from eigenvalues or singular values
     *
     * \f[
     *  \prod_{i=1}^{n}\left({\lambda_{i}}\right)^{\frac{1}{n}}
     * \f]
     */
    T omnivariance;

    /**
     * @brief Entropy from eigenvalues or singular values
     *
     * \f[
     *  -\sum_{i=1}^{n}{\lambda_{i}\ln(\lambda_{i})}
     * \f]
     */
    T entropy;

    /**
     * @brief Verticality defining planes from \f$\mathbb{R}^{3}\f$
     *
     * Let \f$\hat{v}\f$ be the plane orthonormal
     *
     * \f[
     *  ||(v_{1}, v_{2})||
     * \f]
     */
    T verticality;

    /**
     * @brief Horizontality defining planes from \f$\mathbb{R}^{3}\f$
     *
     * Let \f$\hat{v}\f$ be the plane othonormal
     *
     * \f[
     *  |(v_{3}|
     * \f]
     */
    T horizontality;

    /**
     * @brief Linearity defining planes from \f$\mathbb{R}^{3}\f$
     *
     * Let \f$\lambda_{1}\f$ be the minimum eigen value or singular value and
     *  \f$\lambda_{n}\f$ be the maximum eigen value or singular value, so for
     *  \f$\mathbb{R}^{3}, \lambda_{n}=\lambda_{3}\f$
     *
     * \f[
     *  \frac{\lambda_{n}-\lambda_{2}}{\lambda_{n}}
     * \f]
     */
     T linearity;

    /**
     * @brief Planarity defining planes from \f$\mathbb{R}^{3}\f$
     *
     * Let \f$\lambda_{1}\f$ be the minimum eigen value or singular value and
     *  \lambda_{n} be the maximum eigen value or singular value, so for
     *  \f$\mathbb{R}^{3}, \lambda_{n}=\lambda_{3}\f$
     *
     * \f[
     *  \frac{\lambda_{2}-\lambda_{1}}{\lambda_{n}}
     * \f]
     *
     */
     T planarity;

     /**
      * @brief Sphericity defining the plane
     *
     * Let \f$\lambda_{1}\f$ be the minimum eigen value or singular value and
     *  \f$\lambda_{n}\f$ be the maximum eigen value or singular value, so for
     *  \f$\mathbb{R}^{3}, \lambda_{n}=\lambda_{3}\f$
     *
     * \f[
     *  \frac{\lambda_{1}}{\lambda_{n}}
     * \f]
     */
     T sphericity;

     /**
      * @brief Two angular verticalities
      *
      * \f[
      *     V_{1} = \left|\frac{\pi}{2} - \textrm{angle}(e_{1},e_{z})\right|
      * \f]
      *
      * \f[
      *     V_{2} = \left|\frac{\pi}{2} - \textrm{angle}(e_{2},e_{z})\right|
      * \f]
      *
      * Where \f$e_{z}=(0, 0, 1)\f$ for the 3D case
      */
     vector<T> angularVerticality;

     /**
      * @brief Two vertical moments
      *
      * Let \f$n\f$ be the number of neighbors, \f$c\f$ be the centroid and
      *     \f$k\f$ be the moment. In this case \f$k \in {1, 2}\f$.
      * So vertical moments can be defined by following expression:
      *
      * \f[
      *     \frac{1}{n} \sum_{i=1}^{n}{\langle{p_{i}-c, e_{z}}\rangle^{k}}
      * \f]
      *
      * Where \f$e_{z}=(0, 0, 1)\f$ for the 3D case
      *
      */
     vector<T> verticalMoments;

     /**
      * @brief Absolute moments of order 1 and 2 for each eigen or singular
      *     vector defining the plane
      *
      * Let \f$n\f$ be the number of neighbors, \f$c\f$ be the centroid and
      *     \f$k\f$ be the moment. In this case \f$k \in {1, 2}\f$.
      * So absolute moments can be defined by following expression:
      *
      * \f[
      *     \frac{1}{n}\left|
      *         \sum_{i=1}^{n}{\langle{p_{i}-c, e_{j}}\rangle^{k}}
      *     \right|
      * \f]
      *
      * Where \f$e_{j}\f$ is the jth eigen or singular vector
      */
     vector<T> absoluteMoments;

     // ***  CONSTRUCTION / DESTRUCTION  *** //
     // ************************************ //
    /**
     * @brief Default plane constructor
     */
    DetailedPlane() = default;
    /**
     * @brief Build a detailed plane with given centroid, orthonormal and
     *  scatter (plane attributes)
     * @see SurfaceInspector::maths::Plane
     */
    DetailedPlane(
        vector<T> centroid,
        vector<T> orthonormal,
        T scatter=0,
        T curvature=0
    ) :
        Plane<T>(centroid, orthonormal, scatter, curvature)
    {};
    /**
     * @brief Build a detailed plane with all attributes
     *
     * @see SurfaceInspector::maths::Plane
     * @see DetailedPlane::
     */
    DetailedPlane(
        vector<T> centroid,
        vector<T> orthonormal,
        T scatter=0,
        T curvature=0,
        T sum=0,
        T omnivariance=0,
        T entropy=0,
        T verticality=0,
        T horizontality=0,
        T linearity=0,
        T planarity=0,
        T sphericity=0,
        vector<T> angularVerticality=vector<T>(0),
        vector<T> verticalMoments=vector<T>(0),
        vector<T> absoluteMoments=vector<T>(0)
    ) :
        Plane<T>(centroid, orthonormal, scatter, curvature),
        sum(sum),
        omnivariance(omnivariance),
        entropy(entropy),
        verticality(verticality),
        horizontality(horizontality),
        linearity(linearity),
        planarity(planarity),
        sphericity(sphericity),
        angularVerticality(angularVerticality),
        verticalMoments(verticalMoments),
        absoluteMoments(absoluteMoments)
    {};
    virtual ~DetailedPlane() {};
};


}}
