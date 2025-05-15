#ifndef _SURFACEINSPECTOR_MATHS_STATISTICS_HPP_
#include <maths/Statistics.hpp>
#endif

#include <surfaceinspector/maths/Vector.hpp>
#include <surfaceinspector/util/draggers/MinDragger.hpp>
#include <surfaceinspector/util/draggers/MaxDragger.hpp>

using SurfaceInspector::maths::Statistics;
using SurfaceInspector::maths::Vector;
using SurfaceInspector::util::draggers::MinDragger;
using SurfaceInspector::util::draggers::MaxDragger;

// ***  STATIC METHODS  *** //
// ************************ //
template <typename T> T Statistics::mean(vector<T> const &vals){
    T mean = 0;
    for(T const &val : vals) mean += val;
    return mean / ((T)vals.size());
}
template <typename T> T Statistics::variance(vector<T> const &vals){
    T mean = Statistics::mean(vals);
    return Statistics::variance(vals, mean);
}
template <typename T>
T Statistics::variance(vector<T> const &vals, T const mean){
    T variance = 0;
    for(T const &val : vals){
        T diff = mean - val;
        variance += diff * diff;
    }
    return variance / ((T)vals.size());
}
template <typename T>
T Statistics::stdev(vector<T> const &vals){
    T variance = Statistics::variance(vals);
    return Statistics::stdev(variance);
}
template <typename T>
T Statistics::stdev(vector<T> const &vals, T const mean){
    T variance = Statistics::variance(vals, mean);
    return Statistics::stdev(variance);
}
template <typename T>
T Statistics::covariance(
    vector<T> const &X,
    vector<T> const &Y,
    bool besselCorrection
){
    return covariance(X, Y, mean(X), mean(Y), besselCorrection);
}
template <typename T>
T Statistics::covariance(
    vector<T> const &X,
    vector<T> const &Y,
    T xMean,
    T yMean,
    bool besselCorrection
){
    T covar = 0.0;
    size_t n = X.size();
    double N = (double) n;
    if(besselCorrection) N = (double) (n-1);
    for(size_t i = 0 ; i < n ; i++){
        covar += (X[i]-xMean) * (Y[i]-yMean);
    }
    return covar / N;
}
template <typename T>
arma::Mat<T> Statistics::covarianceMatrix(
    vector<vector<T>> dataset,
    bool besselCorrection
){
    // Compute means
    vector<T> means;
    for(vector<T> &vals : dataset) means.push_back(Statistics::mean(vals));

    // Obtain covariance matrix
    return covarianceMatrix(dataset, means, besselCorrection);
}
template <typename T>
arma::Mat<T> Statistics::covarianceMatrix(
    vector<vector<T>> dataset,
    vector<T> means,
    bool besselCorrection
){
    // Compute covariance matrix
    size_t n = dataset.size();
    arma::Mat<T> covars(n, n);
    for(size_t i = 0 ; i < n ; i++){
        for(size_t j = 0 ; j < i ; j++){
            covars[i*n+j] = covars[i+j*n];
        }
        for(size_t j = i ; j < n ; j++) {
            covars[i*n+j] = Statistics::covariance(
                dataset[i],
                dataset[j],
                means[i],
                means[j],
                besselCorrection
            );
        }
    }

    // Return covariance matrix
    return covars;
}

template <typename T> vector<T> Statistics::quantiles(
    arma::Col<T> data,
    size_t nQuantiles
){
    // Prepare quantiles computation
    arma::Col<double> quantiles(nQuantiles);
    double splits = (double) (nQuantiles+1);
    for(size_t i = 1 ; i <= nQuantiles ; i++){
        quantiles(i-1) = ((double)(i)) / splits;
    }

    // Compute quantiles
    arma::vec _Q = arma::quantile(data, quantiles);

    // Translate quantiles to standard vector
    vector<T> Q;
    for(size_t i = 0 ; i < nQuantiles ; i++) Q.push_back(_Q[i]);

    // Return
    return Q;
}

template <typename T> vector<T> Statistics::quantiles(
    vector<T> const &data,
    size_t nQuantiles
){
    size_t n = data.size();
    arma::Col<T> vec(n);
    for(size_t i = 0 ; i < n ; i++) vec[i] = data[i];
    return quantiles(vec, nQuantiles);
}

template <typename T> size_t Statistics::quantileFiltering(
    vector<T> &values,
    size_t nQuantiles,
    double filterFactor
){
    size_t filteredCount = 0;
    vector<T> Q = quantiles<T>(values, nQuantiles);
    T DELTA = Q[nQuantiles-1] - Q[0];
    T minBound = Q[0] - filterFactor * DELTA;
    T maxBound = Q[nQuantiles-1] + filterFactor * DELTA;
    for(size_t i = 0 ; i < values.size() ; i++){
        T v = values[i];
        if(v < minBound || v > maxBound){
            values.erase(values.begin() + i);
            --i;
            ++filteredCount;
        }
    }
    return filteredCount;
}


template <typename T> bool Statistics::findTrustableMin(
    vector<T> const & S,
    size_t const m,
    T const tau,
    T &g,
    size_t * discardsCount
){
    // Initialize discards count if not null
    if(discardsCount != nullptr) *discardsCount = 0;

    // Handle particular case m=1
    if(m==1){
        g = Vector<T>::min(S);
        return true;
    }

    // Compute general case
    MinDragger<T> drg = MinDragger<T>(vector<T>(S)); // To drag min1, min2, ...
    return _findTrustable(S, m, tau, drg, g, discardsCount);
}

template <typename T> bool Statistics::findTrustableMax(
    vector<T> const & S,
    size_t const m,
    T const tau,
    T &g,
    size_t * discardsCount
){
    // Initialize discards count if not null
    if(discardsCount != nullptr) *discardsCount = 0;

    // Handle particular case m=1
    if(m==1){
        g = Vector<T>::max(S);
        return true;
    }

    // Compute general case
    MaxDragger<T> drg = MaxDragger<T>(vector<T>(S)); // To drag max1, max2, ...
    return _findTrustable(S, m, tau, drg, g, discardsCount);
}

// ***  INNER METHODS  *** //
// *********************** //
template <typename T> bool Statistics::_findTrustable(
    vector<T> const & S,
    size_t const m,
    T const tau,
    IDragger<double, vector<double>> &drg,
    T &g,
    size_t * discardsCount
){
    // Prepare variables
    size_t m_1 = m-1; // Cardinality of Delta collection (num. elements)
    size_t m_2 = m_1-1; // m-2
    vector<T> Z(m);  // The Z collection
    vector<T> D; // The Delta collection

    // Check there are enough elements to even try
    if(S.size() < m){
        g = 0;
        return false;
    }

    // Compute Z initial population
    for(size_t i = 0 ; i < m ; ++i) Z[i] = drg.next();
    D = Vector<T>::diff(Z); // Delta as discrete differences of Z

    // Iterative delta check
    size_t nDiscards = 0;
    bool check = false, valid=false;
    while(!check) { // While !check ~ While checking not finished
        check = true; // Check is expected to finish at current iteration
        valid = true; // Current iteration is expected to be valid
        for (ssize_t i = m_2; i >= 0; --i) { // The delta check itself
            if(std::fabs(D[i]) > tau){ // Handle too big difference case
                // Handle logic
                valid = false; // Current check status is false (not valid)
                nDiscards = i+1; // How many discards/removals must be done
                if(nDiscards > S.size())break; // Not enough elements to update
                check = false; // There are enough elements, do further checks

                // Update Z and D (Delta)
                for(size_t j=i; j<m_1; ++j)Z[j-i] = Z[j+1]; // Move to the left
                for(size_t j=m_1-i; j<m; ++j)Z[j]=drg.next();//Add to the right
                D = Vector<T>::diff(Z); // Update discrete differences of Z

                // Increase discards count if not null
                if (discardsCount != nullptr) (*discardsCount) += nDiscards;

                break; // Next delta check outer loop iteration
            }
        }
    }

    // Handle undefined trustable case
    if(D.size()!=m_1 || !valid) {
        g = 0;
        return false;
    }

    // Handle defined trustable case
    g = Z[0];
    return true;
    // --- General case : END

}
