#pragma once

#include <armadillo>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Handle plane fitting operations
 */
class PlaneFitter
{
public:
  // ***  STATIC FUNCTIONS  *** //
  // ************************** //
  /**
   * @brief Modify coordinates at matrix M so it is centered at originWaypoint
   * @tparam T Type of number
   * @param M Matrix of coordinates. Following format is mandatory:<br/>
   * M[0] := Row with X coordinates<br/>
   * M[1] := Row with Y coordinates<br/>
   * M[2] := Row with Z coordinates
   */
  template<typename T>
  static void centerCoordinatesMatrix(arma::Mat<T>& M);

  /**
   * @brief Compute the orthonormal of best fitting plane for given Matrix
   * of coordinates.
   *
   * Best fitting plane orthonormal is computed through singular value
   * decomposition
   * \f[
   *  M_{3_{x}n} = U_{3_{x}r} \Sigma_{r_{x}r} V^{T}_{r_{x}n}
   * \f]
   * Then \f$(U_{1_{x}3} , U_{2_{x}3} , U_{3_{x}3})\f$ it is the orthonormal
   * of best fitting plane.
   *
   * @tparam T Type of number
   * @param M Matrix of coordinates. Following format is mandatory:<br/>
   * M[0] := Row with X coordinates<br/>
   * M[1] := Row with Y coordinates<br/>
   * M[2] := Row with Z coordinates
   * @param center If true, then coordinates will be translated to have
   * originWaypoint as center. Otherwise, they will used as they come.<br/>
   * <b>NOTICE</b> reliable orthonormal computation requires coordinates to
   * be centered at originWaypoint. Hence, center should only be setted to False
   * when input matrix is already centered.
   *
   *
   * @return Orthonormal of best fitting plane
   */
  template<typename T>
  static std::vector<T> bestFittingPlaneOrthoNormal(arma::Mat<T>& M,
                                                    bool center = true);
};

// ***  IMPLEMENTATION  *** //
// ************************ //
/**
 * @brief Center given coordinates matrix to 0 (origin)
 * @param M Matrix of coordinates to be centered
 */
template<typename T>
void
PlaneFitter::centerCoordinatesMatrix(arma::Mat<T>& M)
{
  arma::colvec mins = arma::min(M, 1);
  arma::colvec maxs = arma::max(M, 1);
  arma::colvec centers = (maxs + mins) / 2.0;
  M.each_col() -= centers;
}

/**
 * @brief Compute the best fitting plane and its orto normal vector
 * @param M Matrix which best fitting plane ortho normal shall be obtained
 * @param center Specify if the matrix must be centered (true) or not (false).
 * To obtain best fitting plane for a given coordinates matrix using SVD,
 * it is recommended to center the matrix of coordinates. Otherwise, unexpected
 * outputs might be obtained.
 * @return Components of ortho normal vector of best fitting plane
 */
template<typename T>
std::vector<T>
PlaneFitter::bestFittingPlaneOrthoNormal(arma::Mat<T>& M, bool center)
{
  // Transpose M to originWaypoint
  if (center)
    PlaneFitter::centerCoordinatesMatrix(M);

  // Compute SVD
  arma::mat U;
  arma::vec s;
  arma::mat V;
  arma::svd(U, s, V, M);
  return std::vector<T>({ U[6], U[7], U[8] });
}
