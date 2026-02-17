#include <helios/surfaceinspector/maths/Vector.hpp>

// ***  STATIC FUNCTIONS  *** //
// ************************** //
template<typename T>
std::vector<T>
SurfaceInspector::maths::PlaneFitter::centerCoordinatesMatrix(arma::Mat<T>& M)
{
  // Compute center
  arma::rowvec mins = arma::min(M, 0);
  arma::rowvec maxs = arma::max(M, 0);
  arma::rowvec centers = (maxs + mins) / 2.0;

  // Build centroid vector
  std::vector<T> centroid;
  bool needsCentering = false;
  for (size_t i = 0; i < centers.n_elem; i++) {
    centroid.push_back(centers[i]);
    if (centroid[i] != 0)
      needsCentering = true;
  }

  // Center matrix of coordinates
  if (needsCentering)
    M.each_row() -= centers;

  // Return centroid vector
  return centroid;
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::PlaneFitter::translateToOrigin(arma::Mat<T>& M,
                                                        std::vector<T> center)
{
  // Computer centroid
  std::size_t n = center.size();
  arma::rowvec centroid(n);
  for (std::size_t i = 0; i < n; ++i)
    centroid[i] = center[i];

  // Transpose
  M.each_row() -= centroid;

  // Return centroid vector
  return center;
}

template<typename T>
SurfaceInspector::maths::Plane<T>
SurfaceInspector::maths::PlaneFitter::bestFittingPlaneSVD(arma::Mat<T>& M)
{
  // Transpose M to origin
  std::vector<T> centroid =
    SurfaceInspector::maths::PlaneFitter::centerCoordinatesMatrix(M);

  // Compute SVD
  arma::mat U;
  arma::vec s;
  arma::mat V;
  arma::svd_econ(U, s, V, M, "right", "std");

  // Extract common plane components
  SurfaceInspector::maths::Plane<T> p(centroid, std::vector<T>(0), 0, 0);
  T sum = 0.0;
  extractCommonPlaneComponents(V, s, sum, p);

  // Return
  return p;
}

template<typename T>
SurfaceInspector::maths::Plane<T>
SurfaceInspector::maths::PlaneFitter::bestFittingPlanePCA(arma::Mat<T>& M)
{
  // Transpose M to origin
  std::vector<T> centroid =
    SurfaceInspector::maths::PlaneFitter::centerCoordinatesMatrix(M);

  // Compute PCA
  arma::mat coeff;
  arma::mat score;
  arma::vec latent;
  arma::princomp(coeff, score, latent, M);

  // Extract plane components
  SurfaceInspector::maths::Plane<T> p(centroid, std::vector<T>(0), 0, 0);
  T sum = 0.0;
  extractCommonPlaneComponents(coeff, latent, sum, p);

  // Return
  return p;
}

template<typename T>
SurfaceInspector::maths::Plane<T>
SurfaceInspector::maths::PlaneFitter::bestFittingPlaneFromCovariances(
  arma::Mat<T>& M)
{
  // Compute PCA
  arma::vec eigvals;
  arma::mat eigvecs;
  arma::eig_sym(eigvals, eigvecs, M);
  eigvals = arma::reverse(eigvals);
  eigvecs = arma::reverse(eigvecs, 1);

  // Extract plane components
  SurfaceInspector::maths::Plane<T> p(
    std::vector<T>(0), std::vector<T>(0), 0, 0);
  T sum = 0.0;
  extractCommonPlaneComponents(eigvecs, eigvals, sum, p);

  // Return
  return p;
}

template<typename T>
SurfaceInspector::maths::DetailedPlane<T>
SurfaceInspector::maths::PlaneFitter::bestFittingDetailedPlaneSVD(
  arma::Mat<T>& M,
  std::vector<T> center)
{
  // Transpose M to origin
  std::vector<T> centroid;
  if (center.empty())
    centroid = SurfaceInspector::maths::PlaneFitter::centerCoordinatesMatrix(M);
  else
    centroid =
      SurfaceInspector::maths::PlaneFitter::translateToOrigin(M, center);

  // Compute SVD
  arma::mat U;
  arma::vec s;
  arma::mat V;
  arma::svd_econ(U, s, V, M, "right", "std");

  // Extract plane components
  SurfaceInspector::maths::DetailedPlane<T> p(
    centroid, std::vector<T>(0), 0, 0, 0);
  T sum = 0.0;
  extractCommonPlaneComponents(V, s, sum, p);

  // Extract detailed plane components
  extractDetailedPlaneComponents(M, V, s, sum, p);

  // Return
  return p;
}

template<typename T>
SurfaceInspector::maths::DetailedPlane<T>
SurfaceInspector::maths::PlaneFitter::bestFittingDetailedPlanePCA(
  arma::Mat<T>& M,
  std::vector<T> center)
{
  // Transpose M to origin
  std::vector<T> centroid =
    SurfaceInspector::maths::PlaneFitter::centerCoordinatesMatrix(M);
  if (center.empty())
    centroid = SurfaceInspector::maths::PlaneFitter::centerCoordinatesMatrix(M);
  else
    centroid =
      SurfaceInspector::maths::PlaneFitter::translateToOrigin(M, center);

  // Compute PCA
  arma::mat coeff;
  arma::mat score;
  arma::vec latent;
  arma::princomp(coeff, score, latent, M);

  // Extract plane components
  SurfaceInspector::maths::DetailedPlane<T> p(
    centroid, std::vector<T>(0), 0, 0, 0);
  T sum = 0.0;
  extractCommonPlaneComponents(coeff, latent, sum, p);

  // Extract detailed plane components
  extractDetailedPlaneComponents(M, coeff, latent, sum, p);

  // Return
  return p;
}

// ***  INNER FUNCTIONS  *** //
// ************************* //
template<typename T>
void
SurfaceInspector::maths::PlaneFitter::extractCommonPlaneComponents(
  arma::mat const& A,
  arma::vec const& v,
  T& sum,
  SurfaceInspector::maths::Plane<T>& p)
{
  arma::vec _orthonormal = A.col(A.n_cols - 1);
  for (size_t i = 0; i < _orthonormal.n_elem; ++i) {
    p.orthonormal.push_back(_orthonormal[i]);
  }
  size_t n = v.n_elem;
  p.scatter = v[n - 1];
  for (size_t i = 0; i < n; ++i)
    sum += v[i];
  p.curvature = v[n - 1] / sum;
}

template<typename T>
void
SurfaceInspector::maths::PlaneFitter::extractDetailedPlaneComponents(
  arma::Mat<T> const& M,
  arma::mat const& A,
  arma::vec const& v,
  T const& sum,
  SurfaceInspector::maths::DetailedPlane<T>& p)
{
  // Prepare
  std::vector<T>& orthonormal = p.orthonormal;
  std::size_t n = v.n_elem;

  // Detailed plane descriptors
  p.sum = sum;
  T omnivariance = v[0];
  for (size_t i = 1; i < n; ++i)
    omnivariance *= v[i];
  p.omnivariance = std::pow(omnivariance, 1.0 / ((T)n));
  T entropy = 0;
  for (size_t i = 0; i < n; ++i)
    entropy += std::log(v[i]) * v[i];
  p.entropy = -entropy;
  p.verticality = SurfaceInspector::maths::Vector<T>::norm(
    std::vector<T>({ p.orthonormal[0], p.orthonormal[1] }));
  p.horizontality = std::fabs(orthonormal[2]);
  p.linearity = (v[0] - v[1]) / v[0];
  p.planarity = (v[1] - v[2]) / v[0];
  p.sphericity = v[n - 1] / v[0];

  // Prepare further computations
  std::size_t nPoints = M.n_rows;
  std::size_t m = M.n_cols; // Elements per vector/point
  std::vector<T> ez(m);
  arma::rowvec _ez(m);
  for (size_t i = 0; i < m; ++i) {
    ez[i] = 0.0;
    _ez[i] = 0.0;
  }
  ez[m - 1] = 1.0;
  _ez[m - 1] = 1.0;

  // Angular verticalities
  arma::vec _e1 = A.col(0);
  std::vector<T> e1(m);
  for (size_t i = 0; i < m; ++i)
    e1[i] = _e1[i];
  p.angularVerticality = std::vector<T>(2);
  p.angularVerticality[0] =
    SurfaceInspector::maths::Vector<T>::acuteAngle(e1, ez);
  p.angularVerticality[1] =
    SurfaceInspector::maths::Vector<T>::acuteAngle(orthonormal, ez);

  // Vertical moments
  p.verticalMoments = std::vector<T>(2);
  T vm1 = 0.0;
  T vm2 = 0.0;
  for (size_t i = 0; i < nPoints; ++i) {
    T dotProduct = arma::dot(M.row(i), _ez);
    vm1 += dotProduct;
    vm2 += dotProduct * dotProduct;
  }
  p.verticalMoments[0] = vm1 / ((T)nPoints);
  p.verticalMoments[1] = vm2 / ((T)nPoints);

  // Absolute moments
  p.absoluteMoments = std::vector<T>(2 * m);
  for (size_t j = 0; j < A.n_cols; ++j) {
    T absMoment1 = 0.0;
    T absMoment2 = 0.0;
    for (size_t i = 0; i < nPoints; ++i) {
      T dotProduct = arma::dot(M.row(i), A.col(j));
      absMoment1 += dotProduct;
      absMoment2 += dotProduct * dotProduct;
    }
    absMoment1 = std::fabs(absMoment1) / ((T)nPoints);
    absMoment2 = absMoment2 / ((T)nPoints);
    p.absoluteMoments[j * 2] = absMoment1;
    p.absoluteMoments[j * 2 + 1] = absMoment2;
  }
}
