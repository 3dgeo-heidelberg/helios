#ifndef _SURFACEINSPECTOR_MATHS_DISTANCES_HPP_
#include <surfaceinspector/maths/Distances.hpp>
#endif

#include <cmath>

using SurfaceInspector::maths::Distances;

// ***  STATIC METHODS  *** //
// ************************ //
template <typename T>
T Distances::manhattan(std::vector<T> const &p, std::vector<T> const &q){
    T mdist = 0;
    std::size_t n = p.size();
    for(std::size_t i = 0 ; i < n ; i++){
        if(p[i] > q[i]) mdist += p[i] - q[i];
        else mdist += q[i] - p[i];
    }
    return mdist;
}
template <typename T>
T Distances::euclidean(std::vector<T> const &p, std::vector<T> const &q){
    T edist = 0;
    std::size_t const n = p.size();
    for(std::size_t i = 0 ; i < n ; i++){
        T diff = p[i] - q[i];
        edist += diff*diff;
    }
    return std::sqrt(edist);
}
template <typename T>
T Distances::euclidean(
    T const px, T const py,
    T const qx, T const qy
) {
    T xDiff = px - qx;
    T yDiff = py - qy;
    return std::sqrt(xDiff*xDiff + yDiff*yDiff);
}
template <typename T>
T Distances::euclidean(
    T const px, T const py, T const pz,
    T const qx, T const qy, T const qz
){
    T xDiff = px-qx;
    T yDiff = py-qy;
    T zDiff = pz-qz;
    return std::sqrt(xDiff*xDiff + yDiff*yDiff + zDiff*zDiff);
}
template <typename T>
T Distances::minkowski(int d, std::vector<T> const &p, std::vector<T> const &q){
    if(d == 1) return manhattan(p, q);
    if(d == 2) return euclidean(p, q);
    T mdist = 0;
    std::size_t n =  p.size();
    double exponent = (double) d;
    double rootExponent = 1.0/exponent;
    for(std::size_t i = 0 ; i < n ; i++){
        T diff = std::fabs(p[i] - q[i]);
        mdist += std::pow(diff, exponent);
    }
    return std::pow(mdist, rootExponent);
}
