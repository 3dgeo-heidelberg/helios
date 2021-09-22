#ifndef _SURFACEINSPECTOR_MATHS_SINGULARCONTEXTCOMPUTER_HPP_
#include <surfaceinspector/maths/SingularContextComputer.hpp>
#endif

#include <surfaceinspector/maths/Vector.hpp>
#include <surfaceinspector/maths/Histogram.hpp>
#include <surfaceinspector/util/SurfaceInspectorException.hpp>

using arma::mat;
using arma::vec;
using arma::uvec;

using SurfaceInspector::maths::Vector;
using SurfaceInspector::maths::Histogram;
using SurfaceInspector::maths::SingularContextComputer;

// ***  INITIALIZE  *** //
// ******************** //
template <typename T>
void SingularContextComputer<T>::init(
    bool normalize,
    bool center,
    size_t aWorst,
    size_t aBest,
    size_t bWorst,
    size_t bBest
){
    this->normalizeFlag = normalize;
    this->centerFlag = center;
    this->aWorst = aWorst;
    this->aBest = aBest;
    this->bWorst = bWorst;
    this->bBest = bBest;
    this->normalize = &SingularContextComputer<T>::normalizeDefault;
    this->center = &SingularContextComputer<T>::centerDefault;
}

// ***  MAIN FUNCTIONS  *** //
// ************************ //
template <typename T>
SingularContextDescriptors<T> SingularContextComputer<T>::describe(Mat<T> &M)
const {
    // Preprocess matrix
    if(normalizeFlag) (this->*normalize)(M);
    if(centerFlag) (this->*center)(M);

    // Compute SVD
    arma::mat U;
    arma::vec s;
    arma::mat V;
    if(!arma::svd_econ(U, s, V, M, "right", "std")){
        throw SurfaceInspectorException(
            "Failed to apply singular value decomposition at "
            "SingularContextComputer<T>::describe(Mat<T> &M)"
        );
    }

    // Extract min and max singular values and singular vectors
    SingularContextDescriptors<T> scd;
    size_t r = s.n_elem;
    scd.minSingularValue = s[r-1];
    scd.maxSingularValue = s[0];

    // Obtain indices and components from singular vectors
    arma::vec vWorst = V.col(r-1);
    if(absoluteComponents) vWorst = arma::abs(V.col(r-1));
    if(aWorst>0){
        scd.worstVectorMinIndices = alpha(vWorst, aWorst);
        scd.worstVectorMinComponents = extractComponents(
            vWorst,
            scd.worstVectorMinIndices
        );
    }
    if(bWorst>0){
        scd.worstVectorMaxIndices = beta(vWorst, bWorst);
        scd.worstVectorMaxComponents = extractComponents(
            vWorst,
            scd.worstVectorMaxIndices
        );
    }
    arma::vec vBest = V.col(0);
    if(absoluteComponents) vBest = arma::abs(V.col(0));
    if(aBest>0){
        scd.bestVectorMinIndices = alpha(vBest, aBest);
        scd.bestVectorMinComponents = extractComponents(
            vBest,
            scd.bestVectorMinIndices
        );
    }
    if(bBest>0){
        scd.bestVectorMaxIndices = beta(vBest, bBest);
        scd.bestVectorMaxComponents = extractComponents(
            vBest,
            scd.bestVectorMaxIndices
        );
    }

    // Return
    return scd;
}

// ***  AUXILIAR FUNCTIONS  *** //
// **************************** //
template <typename T>
vector<size_t> SingularContextComputer<T>::alpha(
    vec const &v,
    size_t const a
) const {
    vector<size_t> alpha(0);
    uvec vSorted = arma::sort_index(v, "ascend");
    for(size_t i = 0 ; i < a ; ++i){
        alpha.push_back(vSorted.at(i));
    }
    return alpha;
}
template <typename T>
vector<size_t> SingularContextComputer<T>::beta(
    vec const &v,
    size_t const b
) const {
    vector<size_t> beta(0);
    uvec vSorted = arma::sort_index(v, "descend");
    for(size_t i = 0 ; i < b ; ++i){
        beta.push_back(vSorted.at(i));
    }
    return beta;
}

template <typename T>
vector<T> SingularContextComputer<T>::extractComponents(
    vec const &v, vector<size_t> indices
) const {
    size_t m = indices.size();
    vector<T> w(m);
    for(size_t i = 0 ; i < m ; ++i) w[i] = v.at(indices[i]);
    return w;
}

// ***  NORMALIZATION  *** //
// *********************** //
template <typename T>
void SingularContextComputer<T>::configureNorm(
    vector<T> const &a,
    vector<T> const &b
){
    aNorm = a;
    bNorm = b;
    deltaNorm = Vector<T>::subtract(b, a);
}
template <typename T>
void SingularContextComputer<T>::configureNorm(vector<vector<T>> const &X){
    size_t m = X.size();
    vector<T> a(0);
    vector<T> b(0);
    for(size_t i = 0 ; i < m ; ++i){
        Histogram<T> hist(X[i], 256, true, false);
        a.push_back(hist.findCutPoint(0.05));
        b.push_back(hist.findCutPoint(0.95));
    }
    configureNorm(a, b);
}

template <typename T>
void SingularContextComputer<T>::normalizeDefault(Mat<T> &M) const{
    size_t n = M.n_cols;
    T xmin, xmax, xdelta;
    arma::vec col;
    for(size_t i = 0 ; i < n ; ++i){
        col = M.col(i);
        xmin = col.min();
        xmax = col.max();
        xdelta = xmax - xmin;
        M.col(i) = (col - xmin) / xdelta;
    }
}

template <typename T>
void SingularContextComputer<T>::normalizeRGB(Mat<T> &M) const{
    arma::vec col;
    for(size_t i = 0 ; i < 3 ; ++i){
        col = M.col(i);
        M.col(i) = col / 255.0;
    }
}

template <typename T>
void SingularContextComputer<T>::normalizeHSO(Mat<T> &M) const{
    arma::colvec col;
    // Normalize Horizontality
    // Nothing to do because it is already normalized in [0, 1]

    // Normalize Sum
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Omnivariance
    col = M.col(2);
    M.col(2) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SingularContextComputer<T>::normalizeHLI(Mat<T> &M) const{
    arma::colvec col;
    // Normalize Horizontality
    // Nothing to do because it is already normalized in [0, 1]

    // Normalize Linearity
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Intensity
    col = M.col(2);
    M.col(2) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SingularContextComputer<T>::normalizeLP(Mat<T> &M) const{
    arma::colvec col;
    // Normalize Linearity
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Planarity
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];

}

template <typename T>
void SingularContextComputer<T>::normalizeGdCd(Mat<T> &M) const{
    arma::colvec col;
    // Normalize Ground distance
    col = M.col(0);
    M.col(0) = arma::clamp(col, 0, arma::max(col));

    // Normalize Ceil distance
    col = M.col(1);
    M.col(1) = arma::clamp(col, 0, arma::max(col));
}

template <typename T>
void SingularContextComputer<T>::normalizeGdCdI(Mat<T> &M) const{
    arma::colvec col;
    // Normalize Ground distance
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Ceil distance
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];

    // Normalize Intensity
    col = M.col(2);
    M.col(2) = (arma::clamp(col, aNorm[2], bNorm[2])-aNorm[2]) / deltaNorm[2];
}

template <typename T>
void SingularContextComputer<T>::normalizeHV(Mat<T> &M) const{
    arma::colvec col;
    // Normalize horizontality
    col = M.col(0);
    M.col(0) = arma::clamp(col, 0, 1.0);

    // Normalize verticality
    col = M.col(1);
    M.col(1) = arma::clamp(col, 0, 1.0);
}

template <typename T>
void SingularContextComputer<T>::normalizeSO(Mat<T> &M) const{
    arma::colvec col;
    // Normalize Sum
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Omnivariance
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SingularContextComputer<T>::normalizeLC(Mat<T> &M) const{
    arma::colvec col;
    // Normalize Linearity
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Curvature
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SingularContextComputer<T>::normalizeVC(Mat<T> &M) const {
    arma::colvec col;
    // Normalize Verticality
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Curvature
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SingularContextComputer<T>::normalizePC(Mat<T> &M) const {
    arma::colvec col;
    // Normalize Planarity
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Curvature
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SingularContextComputer<T>::normalizeVR(Mat<T> &M) const {
    arma::colvec col;
    // Normalize Verticality
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Roughness
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SingularContextComputer<T>::normalizeCone(Mat<T> &M)const{
    // Normalize X
    M.col(0) = arma::pow(M.col(0) - arma::mean(M.col(0)), 2);

    // Normalize Y
    M.col(1) = arma::pow(M.col(1) - arma::mean(M.col(1)), 2);

    // Normalize Z
    M.col(2) = -arma::pow(M.col(2) - arma::mean(M.col(2)), 2);
}
template <typename T>
void SingularContextComputer<T>::normalizeHyperbolicParaboloid(Mat<T> &M)const{
    // Normalize X
    M.col(0) = arma::pow(M.col(0) - arma::mean(M.col(0)), 2);

    // Normalize Y
    M.col(1) = -arma::pow(M.col(1) - arma::mean(M.col(1)), 2);

    // Normalize Z
    M.col(2) = -2*(M.col(2) - arma::mean(M.col(2)));
}

// ***  CENTERING  *** //
// ******************* //
template <typename T>
void SingularContextComputer<T>::centerDefault(Mat<T> &M) const {
    // Computer centroid
    size_t n = M.n_cols;
    T m = (T) M.n_rows;
    T mu;
    arma::vec col;
    for(size_t i = 0 ; i < n ; ++i){
        col = M.col(i);
        mu = arma::sum(col) / m;
        M.col(i) = col - mu;
    }
}
