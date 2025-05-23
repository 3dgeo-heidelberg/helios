#include <surfaceinspector/maths/Histogram.hpp>
#include <surfaceinspector/maths/Vector.hpp>

// ***  INITIALIZE  *** //
// ******************** //
template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::init(
    bool normalize,
    bool center,
    std::size_t aWorst,
    std::size_t aBest,
    std::size_t bWorst,
    std::size_t bBest
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
SurfaceInspector::maths::SingularContextDescriptors<T> SurfaceInspector::maths::SingularContextComputer<T>::describe(arma::Mat<T> &M)
const {
    // Preprocess matrix
    if(normalizeFlag) (this->*normalize)(M);
    if(centerFlag) (this->*center)(M);

    // Compute SVD
    arma::mat U;
    arma::vec s;
    arma::mat V;
    if(!arma::svd_econ(U, s, V, M, "right", "std")){
        throw SurfaceInspector::util::SurfaceInspectorException(
            "Failed to apply singular value decomposition at "
            "SingularContextComputer<T>::describe(Mat<T> &M)"
        );
    }

    // Extract min and max singular values and singular vectors
    SurfaceInspector::maths::SingularContextDescriptors<T> scd;
    std::size_t r = s.n_elem;
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
std::vector<size_t> SurfaceInspector::maths::SingularContextComputer<T>::alpha(
    arma::vec const &v,
    std::size_t const a
) const {
    std::vector<size_t> alpha(0);
    arma::uvec vSorted = arma::sort_index(v, "ascend");
    for(std::size_t i = 0 ; i < a ; ++i){
        alpha.push_back(vSorted.at(i));
    }
    return alpha;
}

template <typename T>
std::vector<size_t> SurfaceInspector::maths::SingularContextComputer<T>::beta(
    arma::vec const &v,
    std::size_t const b
) const {
    std::vector<size_t> beta(0);
    arma::uvec vSorted = arma::sort_index(v, "descend");
    for(std::size_t i = 0 ; i < b ; ++i){
        beta.push_back(vSorted.at(i));
    }
    return beta;
}

template <typename T>
std::vector<T> SurfaceInspector::maths::SingularContextComputer<T>::extractComponents(
    arma::vec const &v, std::vector<size_t> indices
) const {
    std::size_t m = indices.size();
    std::vector<T> w(m);
    for(std::size_t i = 0 ; i < m ; ++i) w[i] = v.at(indices[i]);
    return w;
}

// ***  NORMALIZATION  *** //
// *********************** //
template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::configureNorm(
    std::vector<T> const &a,
    std::vector<T> const &b
){
    aNorm = a;
    bNorm = b;
    deltaNorm = Vector<T>::subtract(b, a);
}
template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::configureNorm(std::vector<std::vector<T>> const &X){
    std::size_t m = X.size();
    std::vector<T> a(0);
    std::vector<T> b(0);
    for(std::size_t i = 0 ; i < m ; ++i){
        SurfaceInspector::maths::Histogram<T> hist(X[i], 256, true, false);
        a.push_back(hist.findCutPoint(0.05));
        b.push_back(hist.findCutPoint(0.95));
    }
    configureNorm(a, b);
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeDefault(arma::Mat<T> &M) const{
    std::size_t n = M.n_cols;
    T xmin, xmax, xdelta;
    arma::vec col;
    for(std::size_t i = 0 ; i < n ; ++i){
        col = M.col(i);
        xmin = col.min();
        xmax = col.max();
        xdelta = xmax - xmin;
        M.col(i) = (col - xmin) / xdelta;
    }
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeRGB(arma::Mat<T> &M) const{
    arma::vec col;
    for(std::size_t i = 0 ; i < 3 ; ++i){
        col = M.col(i);
        M.col(i) = col / 255.0;
    }
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeHSO(arma::Mat<T> &M) const{
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
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeHLI(arma::Mat<T> &M) const{
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
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeLP(arma::Mat<T> &M) const{
    arma::colvec col;
    // Normalize Linearity
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Planarity
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];

}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeGdCd(arma::Mat<T> &M) const{
    arma::colvec col;
    // Normalize Ground distance
    col = M.col(0);
    M.col(0) = arma::clamp(col, 0, arma::max(col));

    // Normalize Ceil distance
    col = M.col(1);
    M.col(1) = arma::clamp(col, 0, arma::max(col));
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeGdCdI(arma::Mat<T> &M) const{
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
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeHV(arma::Mat<T> &M) const{
    arma::colvec col;
    // Normalize horizontality
    col = M.col(0);
    M.col(0) = arma::clamp(col, 0, 1.0);

    // Normalize verticality
    col = M.col(1);
    M.col(1) = arma::clamp(col, 0, 1.0);
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeSO(arma::Mat<T> &M) const{
    arma::colvec col;
    // Normalize Sum
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Omnivariance
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeLC(arma::Mat<T> &M) const{
    arma::colvec col;
    // Normalize Linearity
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Curvature
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeVC(arma::Mat<T> &M) const {
    arma::colvec col;
    // Normalize Verticality
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Curvature
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizePC(arma::Mat<T> &M) const {
    arma::colvec col;
    // Normalize Planarity
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Curvature
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeVR(arma::Mat<T> &M) const {
    arma::colvec col;
    // Normalize Verticality
    col = M.col(0);
    M.col(0) = (arma::clamp(col, aNorm[0], bNorm[0])-aNorm[0]) / deltaNorm[0];

    // Normalize Roughness
    col = M.col(1);
    M.col(1) = (arma::clamp(col, aNorm[1], bNorm[1])-aNorm[1]) / deltaNorm[1];
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeCone(arma::Mat<T> &M)const{
    // Normalize X
    M.col(0) = arma::pow(M.col(0) - arma::mean(M.col(0)), 2);

    // Normalize Y
    M.col(1) = arma::pow(M.col(1) - arma::mean(M.col(1)), 2);

    // Normalize Z
    M.col(2) = -arma::pow(M.col(2) - arma::mean(M.col(2)), 2);
}

template <typename T>
void SurfaceInspector::maths::SingularContextComputer<T>::normalizeHyperbolicParaboloid(arma::Mat<T> &M)const{
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
void SurfaceInspector::maths::SingularContextComputer<T>::centerDefault(arma::Mat<T> &M) const {
    // Computer centroid
    std::size_t n = M.n_cols;
    T m = (T) M.n_rows;
    T mu;
    arma::vec col;
    for(std::size_t i = 0 ; i < n ; ++i){
        col = M.col(i);
        mu = arma::sum(col) / m;
        M.col(i) = col - mu;
    }
}
