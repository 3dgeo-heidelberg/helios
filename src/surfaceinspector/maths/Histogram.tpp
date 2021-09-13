#ifndef _SURFACEINSPECTOR_MATHS_HISTOGRAM_HPP_
#include <surfaceinspector/maths/Histogram.hpp>
#endif

#include <cmath>

#include <surfaceinspector/util/SurfaceInspectorException.hpp>
#include <surfaceinspector/maths/Vector.hpp>
#include <surfaceinspector/maths/Statistics.hpp>

using SurfaceInspector::util::SurfaceInspectorException;
using SurfaceInspector::maths::Histogram;
using SurfaceInspector::maths::Vector;
using SurfaceInspector::maths::Statistics;



// *** CONSTRUCTION / DESTRUCTION  *** //
// *********************************** //
template <typename T>
Histogram<T>::Histogram(vector<T> x, size_t n, bool relative, bool density) :
    n(n)
{
    // Extract min and max values
    extractMinMax(x);

    // Compute interval for each bin
    computeBinningIntervals();

    // Count for each bin
    recount(x);

    // Compute relative frequencies
    if(relative) computeRelativeFrequencies();

    // Compute density
    if(density) computeDensity();
}

// ***  HISTOGRAM METHODS  *** //
// *************************** //
template <typename T>
GaussianFunction<T> Histogram<T>::estimateGaussian(){
    throw SurfaceInspectorException(
        "GaussianFunction<T> Histogram<T>::estimateGaussian() is not "
        "implemented yet"
    );
}

template <typename T>
T Histogram<T>::findCutPoint(double p){
    double rsum = 0.0;
    size_t i;
    for(i = 0 ; i < n && rsum < p ; ++i) rsum += r[i];
    return a[i-1];
}

// ***  INNER METHODS  *** //
// *********************** //
template <typename T>
void Histogram<T>::extractMinMax(vector<T> const &x){
    xmin = Vector<T>::min(x);
    xmax = Vector<T>::max(x);
}

template <typename T>
void Histogram<T>::computeBinningIntervals(){
    delta = xmax - xmin;
    step = delta / ((T)n);
    a = vector<T>(0);
    b = vector<T>(0);
    for(size_t i = 0 ; i < n ; ++i){
        a.push_back(xmin + step * i);
        b.push_back(xmin + step * (i+1));
    }
}

template <typename T>
void Histogram<T>::recount(vector<T> const &x){
    m = x.size();
    c = vector<size_t>(n, 0);
    size_t cIdx; // Index of bin which count must be increased
    for(T const &xi : x){
        cIdx = (size_t ) std::floor((xi-xmin)/delta * n);
        if(cIdx == n) cIdx = n-1;
        c[cIdx] += 1;
    }
}

template <typename T>
void Histogram<T>::computeRelativeFrequencies() {
    r = vector<double>(n);
    for(size_t i = 0 ; i < n ; ++i){
        r[i] = c[i] / ((double)m);
    }
}

template <typename T>
void Histogram<T>::computeDensity(){
    // Compute unitary area norm
    norm = 0.0;
    for(size_t i = 0 ; i < n ; ++i) norm += step*c[i];

    // Compute density for each bin
    d = vector<double>(n);
    for(size_t i = 0 ; i < n ; ++i) d[i] = c[i] / norm;
}