#ifndef _SURFACEINSPECTOR_MATHS_PLANEFITTER_HPP_
#define _SURFACEINSPECTOR_MATHS_PLANEFITTER_HPP_

#include <vector>

#include <armadillo>

#include <surfaceinspector/util/Object.hpp>
#include <surfaceinspector/maths/Plane.hpp>
#include <surfaceinspector/maths/DetailedPlane.hpp>

using std::vector;

using arma::Mat;

using SurfaceInspector::util::Object;
using SurfaceInspector::maths::Plane;
using SurfaceInspector::maths::DetailedPlane;

namespace SurfaceInspector { namespace maths{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Handle plane fitting operations
 */
class PlaneFitter : public Object{
public:
    // ***  STATIC FUNCTIONS  *** //
    // ************************** //
    /**
     * @brief Modify coordinates at M so they are centered at origin
     * @tparam T Type of number
     * @param M Matrix of coordinates. Following format is mandatory:
     * <br/>M[0] := First X coordinate
     * <br/>M[1] := Second X coordinate
     * <br/>M[n-1] := Last X coordinate
     * <br/>M[n] := First Y coordinate
     * <br/>M[n+1] := Second Y coordinate
     * <br/>M[2n-1] := Last Y coordinate
     * <br/>M[2n] := First Z coordinate
     * <br/>M[2n+1] := Second Z coordinate
     * <br/>M[3n-1] := Last Z coordinate
     * @return Original centroid
     */
    template <typename T>
    static vector<T> centerCoordinatesMatrix(Mat<T> & M);

    /**
     * @brief Translate M to origin transposing by center point
     * @tparam T Type of number
     * @param M Matrix of coordinates to be translated. It must have the same
     *  format than the one specified for centerCoordinatesMatrix function
     * @param center Reference point to compute the origin of vector subspace
     * @return Center as centroid by convenience
     * @see PlaneFitter::centerCoordinatesMatrix(Mat<T> &M)
     */
    template <typename T>
    static vector<T> translateToOrigin(Mat<T> &M, vector<T> center);

    /**
     * @brief Compute the best fitting plane for given Matrix of coordinates
     *  through singular value decomposition.
     *
     * \f[
     *  M_{n_{x}3} = U_{n_{x}r} \Sigma_{r_{x}r} V^{T}_{r_{x}3}
     * \f]
     *
     * Then \f$(V^{T}_{1,3} , V^{T}_{2,3} , V^{T}_{3,3})\f$ it is the
     *  orthonormal of best fitting plane.
     *
     * @tparam T Type of number
     * @param M Matrix of coordinates. Row-coordinates format is mandatory:
     * <br/>M[0] := First X coordinate
     * <br/>M[1] := Second X coordinate
     * <br/>M[n-1] := Last X coordinate
     * <br/>M[n] := First Y coordinate
     * <br/>M[n+1] := Second Y coordinate
     * <br/>M[2n-1] := Last Y coordinate
     * <br/>M[2n] := First Z coordinate
     * <br/>M[2n+1] := Second Z coordinate
     * <br/>M[3n-1] := Last Z coordinate
     *
     * @return Best fitting plane
     */
    template <typename T>
    static Plane<T> bestFittingPlaneSVD(Mat<T> & M);
    /**
     * @brief Compute the best fitting plane for given Matrix of coordinates
     *  through principal component analysis.
     * @tparam T  Type of number
     * @param M Matrix of coordinates. Row-coordinates format is mandatory:
     * <br/>M[0] := First X coordinate
     * <br/>M[1] := Second X coordinate
     * <br/>M[n-1] := Last X coordinate
     * <br/>M[n] := First Y coordinate
     * <br/>M[n+1] := Second Y coordinate
     * <br/>M[2n-1] := Last Y coordinate
     * <br/>M[2n] := First Z coordinate
     * <br/>M[2n+1] := Second Z coordinate
     * <br/>M[3n-1] := Last Z coordinate
     *
     * @return Best fitting plane
     */
    template <typename T>
    static Plane<T> bestFittingPlanePCA(Mat<T> &M);

    /**
     * @brief Compute the best fitting plane for given Matrix of covariances
     *  through eigen value decomposition
     * @tparam T Type of number
     * @param M Matrix of covariances
     * @return Best fitting plane
     */
    template <typename T>
    static Plane<T> bestFittingPlaneFromCovariances(Mat<T> &M);

    /**
     * @brief Like bestFittingPlaneSVD but returning a DetailedPlane which
     *  provides more information than a simple Plane object
     * @param center When center is a non empty vector, it will be used as
     *  the origin of the vector subspace. In consequence, the matrix M
     *  will be treated as an affine space such that when transposed by center
     *  it will be a vector subspace
     * @see PlaneFitter::bestFittingPlaneSVD
     * @see SurfaceInspector::maths::DetailedPlane
     */
    template <typename T>
    static DetailedPlane<T> bestFittingDetailedPlaneSVD(
        Mat<T> &M,
        vector<T> center
    );
    /**
     * @brief Like bestFittingPlanePCA but returning a DetailedPlane which
     *  provides more information than a simple Plane object
     * @param center When center is a non empty vector, it will be used as
     *  the origin of the vector subspace. In consequence, the matrix M
     *  will be treated as an affine space such that when transposed by center
     *  it will be a vector subspace
     * @see PlaneFitter::bestFittingPlanePCA
     * @see SurfaceInspector::maths::DetailedPlane
     */
    template <typename T>
    static DetailedPlane<T> bestFittingDetailedPlanePCA(
        Mat<T> &M,
        vector<T> center
    );

protected:
    // ***  INNER FUNCTIONS  *** //
    // ************************* //
    /**
     * @brief Extract common plane components
     * @param[in] A Matrix containing eigen vectors or singular vectors
     * @param[in] v Vector of singular values or eigen values
     * @param[out] sum Where summation of singular or eigen values will be
     *  stored. It must be passed with 0 as initial value.
     * @param[out] p Plane where extracted components will be stored
     */
    template <typename T>
    static void extractCommonPlaneComponents(
        arma::mat const &A,
        arma::vec const &v,
        T &sum,
        Plane<T> &p
    );

    /**
     * @brief Extract detailed plane components. It must be invoked after
     *  common plane components are extracted into given plane
     * @param[in] M Matrix of coordinates
     * @param[in] A Matrix containing eigen vectors or singular vectors
     * @param[in] v Vector of singular values or eigen values
     * @param[in] sum Ist must contain summation of singular or eigen values
     * @param p Plane where common components are stored and where detailed
     *  components will be placed. It is both an input/output argument
     * @see PlaneFitter::extractCommonPlaneComponents
     */
    template <typename T>
    static void extractDetailedPlaneComponents(
        Mat<T> const &M,
        arma::mat const &A,
        arma::vec const &v,
        T const &sum,
        DetailedPlane<T> &p
    );

};

}}

#include <surfaceinspector/maths/PlaneFitter.tpp>

#endif
