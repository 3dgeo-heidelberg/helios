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
Histogram<T>::Histogram(std::vector<T> x, std::size_t n, bool relative, bool density) :
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

template <typename T>
Histogram<T>::Histogram(
    T xmin,
    T xmax,
    std::vector<T> x,
    std::size_t n,
    bool relative,
    bool density
) :
    n(n),
    xmin(xmin),
    xmax(xmax)
{
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

template <typename T>
size_t Histogram<T>::absCumsum(size_t const start, size_t const end){
    size_t cumsum = 0;
    for(size_t i = start ; i < end ; ++i) cumsum += c[i];
    return cumsum;
}

// ***  INNER METHODS  *** //
// *********************** //
template <typename T>
void Histogram<T>::extractMinMax(std::vector<T> const &x){
    xmin = Vector<T>::min(x);
    xmax = Vector<T>::max(x);
}

template <typename T>
void Histogram<T>::computeBinningIntervals(){
    delta = xmax - xmin;
    step = delta / ((T)n);
    a = std::vector<T>(0);
    b = std::vector<T>(0);
    for(std::size_t i = 0 ; i < n ; ++i){
        a.push_back(xmin + step * i);
        b.push_back(xmin + step * (i+1));
    }
}

template <typename T>
void Histogram<T>::recount(std::vector<T> const &x){
    m = x.size();
    c = std::vector<std::size_t>(n, 0);
    std::size_t cIdx; // Index of bin which count must be increased
    for(T const &xi : x){
        cIdx = (std::size_t) std::floor((xi-xmin)/delta * n);
        if(cIdx >= n) cIdx = n-1;
        c[cIdx] += 1;
    }
}

template <typename T>
void Histogram<T>::computeRelativeFrequencies() {
    r = std::vector<double>(n);
    for(std::size_t i = 0 ; i < n ; ++i){
        r[i] = c[i] / ((double)m);
    }
}

template <typename T>
void Histogram<T>::computeDensity(){
    // Compute unitary area norm
    norm = 0.0;
    for(std::size_t i = 0 ; i < n ; ++i) norm += step*c[i];

    // Compute density for each bin
    d = std::vector<double>(n);
    for(std::size_t i = 0 ; i < n ; ++i) d[i] = c[i] / norm;
}
